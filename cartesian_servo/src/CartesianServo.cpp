#include "CartesianServo.h"
#include <rtt/Component.hpp>
#include <cmath>
#include <stdexcept>

#define CS_LOG_ERR RTT::log(RTT::Error) << "[CartesianServo] "
#define CS_LOG     RTT::log(RTT::Error) << "[CartesianServo] "

// =============================================================================
CartesianServo::CartesianServo(const std::string& name)
    : RTT::TaskContext(name, PreOperational)
    , state_(IDLE)
    , first_iteration_(true)
    , first_fk_received_(false)
    , update_count_(0)
    , smoothing_alpha_(0.85)
    , decel_smoothing_alpha_(0.98)
    , watchdog_timeout_ms_(100.0)
    , max_linear_speed_(0.5)
    , max_angular_speed_(1.0)
    , vel_filter_alpha_(0.8)
    , acc_filter_alpha_(0.9)
    , idle_velocity_threshold_(0.001)
    , dt_(0.002)
    , torque_mode_(false)
    , debug_(false)
{
    // EventPort: FK writes at 500Hz -> triggers updateHook (no setActivity needed)
    this->ports()->addEventPort("CartesianPosition", port_cartesian_position_);

    this->ports()->addPort("CartesianCommand",       port_cartesian_command_);
    this->ports()->addPort("JointPosition",          port_joint_position_);
    this->ports()->addPort("IkOutputJointPosition",  port_ik_output_);
    this->ports()->addPort("IkFailure",              port_ik_failure_);
    this->ports()->addPort("CancelServo",            port_cancel_servo_);

    this->ports()->addPort("new_smoothing_alpha",    port_new_smoothing_alpha_);
    this->ports()->addPort("new_max_linear_speed",   port_new_max_linear_speed_);
    this->ports()->addPort("new_max_angular_speed",  port_new_max_angular_speed_);
    this->ports()->addPort("new_watchdog_timeout",   port_new_watchdog_timeout_);
    this->ports()->addPort("new_torque_mode",        port_new_torque_mode_);
    this->ports()->addPort("new_debug",              port_new_debug_);

    this->ports()->addPort("CartesianPositionCommand",  port_cartesian_position_command_);
    this->ports()->addPort("CartesianVelocity",         port_cartesian_velocity_);
    this->ports()->addPort("CommandJointVelocity",      port_command_joint_velocity_);
    this->ports()->addPort("CommandJointAcceleration",  port_command_joint_acceleration_);
    this->ports()->addPort("ServoActive",               port_servo_active_);
    this->ports()->addPort("ServoStatus",               port_servo_status_);

    this->addProperty("smoothing_alpha",       smoothing_alpha_);
    this->addProperty("decel_smoothing_alpha", decel_smoothing_alpha_);
    this->addProperty("watchdog_timeout_ms",   watchdog_timeout_ms_);
    this->addProperty("max_linear_speed",      max_linear_speed_);
    this->addProperty("max_angular_speed",     max_angular_speed_);
    this->addProperty("vel_filter_alpha",      vel_filter_alpha_);
    this->addProperty("acc_filter_alpha",      acc_filter_alpha_);
    this->addProperty("idle_velocity_threshold", idle_velocity_threshold_);
    this->addProperty("dt",                    dt_);
    this->addProperty("torque_mode",           torque_mode_);
    this->addProperty("debug",                 debug_);

    CS_LOG << "constructor OK, CartesianPosition is EventPort" << RTT::endlog();
}

CartesianServo::~CartesianServo() {}

// =============================================================================
bool CartesianServo::configureHook() {
    CS_LOG << "configureHook: start" << RTT::endlog();

    try {
        cart_vel_msg_.data.resize(6, 0.0);
        CS_LOG << "configureHook: cart_vel_msg OK" << RTT::endlog();

        prev_ik_output_ = Eigen::VectorXd::Zero(CS_DOF);
        prev_dq_        = Eigen::VectorXd::Zero(CS_DOF);
        dq_filtered_    = Eigen::VectorXd::Zero(CS_DOF);
        ddq_filtered_   = Eigen::VectorXd::Zero(CS_DOF);
        CS_LOG << "configureHook: Eigen vectors OK" << RTT::endlog();

        geometry_msgs::Pose sp;
        sp.position.x = sp.position.y = sp.position.z = 0;
        sp.orientation.x = sp.orientation.y = sp.orientation.z = 0;
        sp.orientation.w = 1;
        port_cartesian_position_command_.setDataSample(sp);
        CS_LOG << "configureHook: Pose sample OK" << RTT::endlog();

        port_cartesian_velocity_.setDataSample(cart_vel_msg_);
        CS_LOG << "configureHook: vel sample OK" << RTT::endlog();

        Eigen::VectorXd sv = Eigen::VectorXd::Zero(CS_DOF);
        port_command_joint_velocity_.setDataSample(sv);
        port_command_joint_acceleration_.setDataSample(sv);
        CS_LOG << "configureHook: joint samples OK" << RTT::endlog();

        port_servo_active_.setDataSample(servo_active_msg_);
        port_servo_status_.setDataSample(servo_status_msg_);
        CS_LOG << "configureHook: status samples OK" << RTT::endlog();

    } catch (const std::exception& e) {
        CS_LOG_ERR << "configureHook EXCEPTION: " << e.what() << RTT::endlog();
        return false;
    } catch (...) {
        CS_LOG_ERR << "configureHook UNKNOWN EXCEPTION" << RTT::endlog();
        return false;
    }

    CS_LOG << "configureHook: DONE, ports=" << this->ports()->getPortNames().size()
           << RTT::endlog();
    return true;
}

// =============================================================================
bool CartesianServo::startHook() {
    CS_LOG << "startHook: begin" << RTT::endlog();

    try {
        smooth_output_.position.x = 0;
        smooth_output_.position.y = 0;
        smooth_output_.position.z = 0;
        smooth_output_.orientation.x = 0;
        smooth_output_.orientation.y = 0;
        smooth_output_.orientation.z = 0;
        smooth_output_.orientation.w = 1;
        first_fk_received_ = false;

        if (port_cartesian_position_.read(smooth_output_) != RTT::NoData) {
            first_fk_received_ = true;
            CS_LOG << "startHook: FK data available at start" << RTT::endlog();
        } else {
            CS_LOG << "startHook: no FK data yet, will init from first reading"
                   << RTT::endlog();
        }

        target_pose_ = smooth_output_;
        prev_smooth_output_ = smooth_output_;

        smooth_quat_ = Eigen::Quaterniond(
            smooth_output_.orientation.w,
            smooth_output_.orientation.x,
            smooth_output_.orientation.y,
            smooth_output_.orientation.z);

        double qnorm = smooth_quat_.norm();
        if (qnorm < 1e-10 || !std::isfinite(qnorm)) {
            CS_LOG << "startHook: quaternion norm=" << qnorm
                   << ", resetting to identity" << RTT::endlog();
            smooth_quat_ = Eigen::Quaterniond::Identity();
        } else {
            smooth_quat_.normalize();
        }
        prev_smooth_quat_ = smooth_quat_;

        dq_filtered_.setZero();
        ddq_filtered_.setZero();
        prev_dq_.setZero();

        if (port_joint_position_.read(prev_ik_output_) != RTT::NewData) {
            prev_ik_output_.setZero();
        }

        first_iteration_ = true;
        update_count_ = 0;
        state_ = IDLE;
        last_command_time_ = rtt_rosclock::host_now();

        servo_active_msg_.data = false;
        port_servo_active_.write(servo_active_msg_);
        servo_status_msg_.data = 0;
        port_servo_status_.write(servo_status_msg_);

    } catch (const std::exception& e) {
        CS_LOG_ERR << "startHook EXCEPTION: " << e.what() << RTT::endlog();
        return false;
    } catch (...) {
        CS_LOG_ERR << "startHook UNKNOWN EXCEPTION" << RTT::endlog();
        return false;
    }

    CS_LOG << "startHook: OK, alpha=" << smoothing_alpha_
           << " max_v=" << max_linear_speed_
           << " dt=" << dt_ << RTT::endlog();
    return true;
}

// =============================================================================
void CartesianServo::updateHook() {
    update_count_++;

    // --- SECTION 1: Read FK (EventPort trigger) ---
    try {
        geometry_msgs::Pose fk_pose;
        RTT::FlowStatus fs = port_cartesian_position_.read(fk_pose);
        if (fs == RTT::NewData && !first_fk_received_) {
            if (isFinitePose(fk_pose)) {
                smooth_output_ = fk_pose;
                prev_smooth_output_ = fk_pose;
                target_pose_ = fk_pose;
                smooth_quat_ = Eigen::Quaterniond(
                    fk_pose.orientation.w, fk_pose.orientation.x,
                    fk_pose.orientation.y, fk_pose.orientation.z);
                double qn = smooth_quat_.norm();
                if (qn < 1e-10 || !std::isfinite(qn)) {
                    smooth_quat_ = Eigen::Quaterniond::Identity();
                } else {
                    smooth_quat_.normalize();
                }
                prev_smooth_quat_ = smooth_quat_;
                first_fk_received_ = true;
                CS_LOG << "updateHook[" << update_count_ << "]: FK init at ["
                       << fk_pose.position.x << ", " << fk_pose.position.y
                       << ", " << fk_pose.position.z << "]" << RTT::endlog();
            } else {
                CS_LOG_ERR << "updateHook[" << update_count_
                           << "]: FK pose has NaN/Inf, ignoring" << RTT::endlog();
            }
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 1 (FK read) EXCEPTION: "
                   << e.what() << RTT::endlog();
        return;
    }

    // Not initialized yet — skip everything
    if (!first_fk_received_) {
        if (update_count_ % 500 == 0) {
            CS_LOG << "updateHook[" << update_count_
                   << "]: waiting for FK data..." << RTT::endlog();
        }
        return;
    }

    // --- SECTION 2: Read parameter updates ---
    try {
        readParameterUpdates();
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 2 (params) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 3: Read Cartesian command ---
    try {
        geometry_msgs::Pose new_cmd;
        if (RTT::NewData == port_cartesian_command_.read(new_cmd)) {
            if (isFinitePose(new_cmd)) {
                target_pose_ = new_cmd;
                last_command_time_ = rtt_rosclock::host_now();
                state_ = TRACKING;
            }
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 3 (command) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 4: First iteration seed ---
    try {
        if (first_iteration_) {
            Eigen::VectorXd ik_seed;
            if (port_ik_output_.read(ik_seed) == RTT::NewData) {
                prev_ik_output_ = ik_seed;
            }
            first_iteration_ = false;
            CS_LOG << "updateHook: first iteration done" << RTT::endlog();
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 4 (seed) EXCEPTION: "
                   << e.what() << RTT::endlog();
        first_iteration_ = false;
    }

    // --- SECTION 5: Cancel/stop ---
    try {
        std_msgs::Int32 cancel;
        if (RTT::NewData == port_cancel_servo_.read(cancel) && cancel.data != 0) {
            if (state_ == TRACKING) state_ = DECELERATING;
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 5 (cancel) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 6: IK failure ---
    try {
        std_msgs::Bool ik_fail;
        if (RTT::NewData == port_ik_failure_.read(ik_fail) && ik_fail.data) {
            if (state_ == TRACKING) state_ = DECELERATING;
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 6 (IK fail) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 7: Watchdog ---
    try {
        if (state_ == TRACKING) {
            double elapsed_ms = (rtt_rosclock::host_now() - last_command_time_).toSec() * 1000.0;
            if (std::isfinite(elapsed_ms) && elapsed_ms > watchdog_timeout_ms_) {
                state_ = DECELERATING;
            }
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 7 (watchdog) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 8: Smoothing ---
    try {
        double alpha;
        switch (state_) {
            case IDLE:          alpha = 1.0; break;
            case TRACKING:      alpha = smoothing_alpha_; break;
            case DECELERATING:  alpha = decel_smoothing_alpha_;
                                target_pose_ = smooth_output_; break;
        }

        // position
        smooth_output_.position.x = alpha * smooth_output_.position.x
                                  + (1.0 - alpha) * target_pose_.position.x;
        smooth_output_.position.y = alpha * smooth_output_.position.y
                                  + (1.0 - alpha) * target_pose_.position.y;
        smooth_output_.position.z = alpha * smooth_output_.position.z
                                  + (1.0 - alpha) * target_pose_.position.z;

        // NaN guard on position
        if (!std::isfinite(smooth_output_.position.x) ||
            !std::isfinite(smooth_output_.position.y) ||
            !std::isfinite(smooth_output_.position.z)) {
            CS_LOG_ERR << "updateHook[" << update_count_
                       << "]: NaN in smooth position! Resetting to prev"
                       << RTT::endlog();
            smooth_output_ = prev_smooth_output_;
        }

        // orientation SLERP
        Eigen::Quaterniond q_target(
            target_pose_.orientation.w, target_pose_.orientation.x,
            target_pose_.orientation.y, target_pose_.orientation.z);

        double qt_norm = q_target.norm();
        if (qt_norm < 1e-10 || !std::isfinite(qt_norm)) {
            q_target = smooth_quat_;  // keep current if target is garbage
        } else {
            q_target.normalize();
        }

        if (smooth_quat_.dot(q_target) < 0.0) {
            q_target = Eigen::Quaterniond(
                -q_target.w(), -q_target.x(), -q_target.y(), -q_target.z());
        }

        smooth_quat_ = smooth_quat_.slerp(1.0 - alpha, q_target);

        double sq_norm = smooth_quat_.norm();
        if (sq_norm < 1e-10 || !std::isfinite(sq_norm)) {
            CS_LOG_ERR << "updateHook[" << update_count_
                       << "]: NaN in smooth_quat! norm=" << sq_norm
                       << " Resetting to prev" << RTT::endlog();
            smooth_quat_ = prev_smooth_quat_;
        } else {
            smooth_quat_.normalize();
        }

        smooth_output_.orientation.w = smooth_quat_.w();
        smooth_output_.orientation.x = smooth_quat_.x();
        smooth_output_.orientation.y = smooth_quat_.y();
        smooth_output_.orientation.z = smooth_quat_.z();

    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 8 (smoothing) EXCEPTION: "
                   << e.what() << RTT::endlog();
        smooth_output_ = prev_smooth_output_;
        smooth_quat_ = prev_smooth_quat_;
    }

    // --- SECTION 9: Velocity estimation ---
    double vx = 0, vy = 0, vz = 0, linear_speed = 0;
    double angular_speed = 0;
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();

    try {
        if (dt_ > 1e-9) {
            vx = (smooth_output_.position.x - prev_smooth_output_.position.x) / dt_;
            vy = (smooth_output_.position.y - prev_smooth_output_.position.y) / dt_;
            vz = (smooth_output_.position.z - prev_smooth_output_.position.z) / dt_;
        }
        if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) {
            linear_speed = std::sqrt(vx*vx + vy*vy + vz*vz);
        } else {
            vx = vy = vz = 0; linear_speed = 0;
        }

        // angular velocity
        Eigen::Quaterniond dq = smooth_quat_ * prev_smooth_quat_.inverse();
        double dq_norm = dq.norm();
        if (dq_norm > 1e-10 && std::isfinite(dq_norm) && dt_ > 1e-9) {
            dq.normalize();
            double angle = 2.0 * std::acos(std::min(1.0, std::abs(dq.w())));
            if (angle > 1e-10) {
                Eigen::AngleAxisd aa(dq);
                omega = aa.axis() * aa.angle() / dt_;
            }
        }
        angular_speed = omega.norm();
        if (!std::isfinite(angular_speed)) {
            omega.setZero();
            angular_speed = 0;
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 9 (velocity) EXCEPTION: "
                   << e.what() << RTT::endlog();
        vx = vy = vz = linear_speed = angular_speed = 0;
        omega.setZero();
    }

    // --- SECTION 10: Velocity clamping ---
    try {
        if (linear_speed > max_linear_speed_ && linear_speed > 1e-9) {
            double scale = max_linear_speed_ / linear_speed;
            smooth_output_.position.x = prev_smooth_output_.position.x
                + (smooth_output_.position.x - prev_smooth_output_.position.x) * scale;
            smooth_output_.position.y = prev_smooth_output_.position.y
                + (smooth_output_.position.y - prev_smooth_output_.position.y) * scale;
            smooth_output_.position.z = prev_smooth_output_.position.z
                + (smooth_output_.position.z - prev_smooth_output_.position.z) * scale;
            if (dt_ > 1e-9) {
                vx = (smooth_output_.position.x - prev_smooth_output_.position.x) / dt_;
                vy = (smooth_output_.position.y - prev_smooth_output_.position.y) / dt_;
                vz = (smooth_output_.position.z - prev_smooth_output_.position.z) / dt_;
            }
            linear_speed = max_linear_speed_;
        }
        // angular clamping
        if (angular_speed > max_angular_speed_ && angular_speed > 1e-9) {
            double scale = max_angular_speed_ / angular_speed;
            smooth_quat_ = prev_smooth_quat_.slerp(scale, smooth_quat_);
            double n = smooth_quat_.norm();
            if (n > 1e-10 && std::isfinite(n)) smooth_quat_.normalize();
            smooth_output_.orientation.w = smooth_quat_.w();
            smooth_output_.orientation.x = smooth_quat_.x();
            smooth_output_.orientation.y = smooth_quat_.y();
            smooth_output_.orientation.z = smooth_quat_.z();
            angular_speed = max_angular_speed_;
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 10 (clamp) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 11: State transition ---
    try {
        if (state_ == DECELERATING && linear_speed < idle_velocity_threshold_) {
            state_ = IDLE;
            servo_active_msg_.data = false;
            port_servo_active_.write(servo_active_msg_);
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 11 (transition) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 12: Write outputs ---
    try {
        port_cartesian_position_command_.write(smooth_output_);

        cart_vel_msg_.data[0] = vx;
        cart_vel_msg_.data[1] = vy;
        cart_vel_msg_.data[2] = vz;
        cart_vel_msg_.data[3] = omega.x();
        cart_vel_msg_.data[4] = omega.y();
        cart_vel_msg_.data[5] = omega.z();
        port_cartesian_velocity_.write(cart_vel_msg_);

        servo_status_msg_.data = (int)state_;
        port_servo_status_.write(servo_status_msg_);

        if (state_ == TRACKING) {
            servo_active_msg_.data = true;
            port_servo_active_.write(servo_active_msg_);
        }
    } catch (const std::exception& e) {
        CS_LOG_ERR << "updateHook SECTION 12 (write) EXCEPTION: "
                   << e.what() << RTT::endlog();
    }

    // --- SECTION 13: Torque mode ---
    if (torque_mode_) {
        try {
            Eigen::VectorXd ik_out(CS_DOF);
            if (port_ik_output_.read(ik_out) == RTT::NewData && dt_ > 1e-9) {
                Eigen::VectorXd dq_raw  = (ik_out - prev_ik_output_) / dt_;
                Eigen::VectorXd ddq_raw = (dq_raw - prev_dq_) / dt_;
                for (size_t i = 0; i < CS_DOF; i++) {
                    if (std::isfinite(dq_raw[i]))
                        dq_filtered_[i] = vel_filter_alpha_ * dq_filtered_[i]
                                        + (1.0 - vel_filter_alpha_) * dq_raw[i];
                    if (std::isfinite(ddq_raw[i]))
                        ddq_filtered_[i] = acc_filter_alpha_ * ddq_filtered_[i]
                                         + (1.0 - acc_filter_alpha_) * ddq_raw[i];
                }
                prev_ik_output_ = ik_out;
                prev_dq_ = dq_raw;
            }
            port_command_joint_velocity_.write(dq_filtered_);
            port_command_joint_acceleration_.write(ddq_filtered_);
        } catch (const std::exception& e) {
            CS_LOG_ERR << "updateHook SECTION 13 (torque) EXCEPTION: "
                       << e.what() << RTT::endlog();
        }
    }

    // --- SECTION 14: Store previous ---
    prev_smooth_output_ = smooth_output_;
    prev_smooth_quat_ = smooth_quat_;

    // periodic log (every 5 seconds at ~500Hz)
    if (update_count_ % 2500 == 0) {
        CS_LOG << "updateHook[" << update_count_ << "]: alive, state="
               << (int)state_ << " pos=["
               << smooth_output_.position.x << ","
               << smooth_output_.position.y << ","
               << smooth_output_.position.z << "] v="
               << linear_speed << " m/s" << RTT::endlog();
    }
}

// =============================================================================
void CartesianServo::stopHook() {
    state_ = IDLE;
    servo_active_msg_.data = false;
    port_servo_active_.write(servo_active_msg_);
    CS_LOG << "stopHook: stopped after " << update_count_ << " cycles" << RTT::endlog();
}

// =============================================================================
void CartesianServo::readParameterUpdates() {
    std_msgs::Float32MultiArray msg;
    if (RTT::NewData == port_new_smoothing_alpha_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v >= 0.0 && v < 1.0) smoothing_alpha_ = v;
    }
    if (RTT::NewData == port_new_max_linear_speed_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v > 0.0) max_linear_speed_ = v;
    }
    if (RTT::NewData == port_new_max_angular_speed_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v > 0.0) max_angular_speed_ = v;
    }
    if (RTT::NewData == port_new_watchdog_timeout_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v > 0.0) watchdog_timeout_ms_ = v;
    }
    std_msgs::Bool bmsg;
    if (RTT::NewData == port_new_torque_mode_.read(bmsg)) torque_mode_ = bmsg.data;
    if (RTT::NewData == port_new_debug_.read(bmsg)) debug_ = bmsg.data;
}

bool CartesianServo::isFinitePose(const geometry_msgs::Pose& p) const {
    return std::isfinite(p.position.x) && std::isfinite(p.position.y)
        && std::isfinite(p.position.z)
        && std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y)
        && std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
}

bool CartesianServo::isFiniteQuat(const Eigen::Quaterniond& q) const {
    return std::isfinite(q.w()) && std::isfinite(q.x())
        && std::isfinite(q.y()) && std::isfinite(q.z()) && q.norm() > 1e-10;
}

ORO_CREATE_COMPONENT(CartesianServo)
