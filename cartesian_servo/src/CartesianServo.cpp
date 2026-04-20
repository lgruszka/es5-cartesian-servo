#include "CartesianServo.h"

#include <rtt/Component.hpp>
#include <cmath>

// =============================================================================
// Constructor
// =============================================================================

CartesianServo::CartesianServo(const std::string& name)
    : RTT::TaskContext(name, PreOperational)
    , state_(IDLE)
    , first_iteration_(true)
    // defaults
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
    // --- Input ports ---
    this->ports()->addPort("CartesianCommand",       port_cartesian_command_);
    this->ports()->addPort("CartesianPosition",      port_cartesian_position_);
    this->ports()->addPort("JointPosition",          port_joint_position_);
    this->ports()->addPort("IkOutputJointPosition",  port_ik_output_);
    this->ports()->addPort("IkFailure",              port_ik_failure_);
    this->ports()->addPort("CancelServo",            port_cancel_servo_);

    // online parameter update ports
    this->ports()->addPort("new_smoothing_alpha",    port_new_smoothing_alpha_);
    this->ports()->addPort("new_max_linear_speed",   port_new_max_linear_speed_);
    this->ports()->addPort("new_max_angular_speed",  port_new_max_angular_speed_);
    this->ports()->addPort("new_watchdog_timeout",   port_new_watchdog_timeout_);
    this->ports()->addPort("new_torque_mode",        port_new_torque_mode_);
    this->ports()->addPort("new_debug",              port_new_debug_);

    // --- Output ports ---
    this->ports()->addPort("CartesianPositionCommand",  port_cartesian_position_command_);
    this->ports()->addPort("CartesianVelocity",         port_cartesian_velocity_);
    this->ports()->addPort("CommandJointVelocity",      port_command_joint_velocity_);
    this->ports()->addPort("CommandJointAcceleration",  port_command_joint_acceleration_);
    this->ports()->addPort("ServoActive",               port_servo_active_);
    this->ports()->addPort("ServoStatus",               port_servo_status_);

    // --- Properties ---
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
}

CartesianServo::~CartesianServo() {}

// =============================================================================
// configureHook
// =============================================================================

bool CartesianServo::configureHook() {
    // pre-allocate output messages
    cart_vel_msg_.data.resize(6, 0.0);

    // pre-allocate Eigen vectors for torque mode
    prev_ik_output_ = Eigen::VectorXd::Zero(CS_DOF);
    prev_dq_        = Eigen::VectorXd::Zero(CS_DOF);
    dq_filtered_    = Eigen::VectorXd::Zero(CS_DOF);
    ddq_filtered_   = Eigen::VectorXd::Zero(CS_DOF);

    // set data samples for output ports
    geometry_msgs::Pose sample_pose;
    port_cartesian_position_command_.setDataSample(sample_pose);

    port_cartesian_velocity_.setDataSample(cart_vel_msg_);

    Eigen::VectorXd sample_vec = Eigen::VectorXd::Zero(CS_DOF);
    port_command_joint_velocity_.setDataSample(sample_vec);
    port_command_joint_acceleration_.setDataSample(sample_vec);

    port_servo_active_.setDataSample(servo_active_msg_);
    port_servo_status_.setDataSample(servo_status_msg_);

    return true;
}

// =============================================================================
// startHook
// =============================================================================

bool CartesianServo::startHook() {
    // initialize from current FK pose
    if (port_cartesian_position_.read(smooth_output_) == RTT::NoData) {
        RTT::log(RTT::Error) << "[CartesianServo] No FK data at startup — "
                             << "cannot initialize. Is FK running?"
                             << RTT::endlog();
        return false;
    }

    target_pose_       = smooth_output_;
    prev_smooth_output_ = smooth_output_;

    // initialize quaternion state
    smooth_quat_ = Eigen::Quaterniond(
        smooth_output_.orientation.w,
        smooth_output_.orientation.x,
        smooth_output_.orientation.y,
        smooth_output_.orientation.z);
    smooth_quat_.normalize();
    prev_smooth_quat_ = smooth_quat_;

    // zero velocity/acceleration state
    dq_filtered_.setZero();
    ddq_filtered_.setZero();
    prev_dq_.setZero();

    // seed IK differentiation from current joint position
    if (port_joint_position_.read(prev_ik_output_) != RTT::NewData) {
        prev_ik_output_.setZero();
    }

    first_iteration_ = true;
    state_ = IDLE;
    last_command_time_ = rtt_rosclock::host_now();

    // publish initial status
    servo_active_msg_.data = false;
    port_servo_active_.write(servo_active_msg_);
    servo_status_msg_.data = 0;
    port_servo_status_.write(servo_status_msg_);

    RTT::log(RTT::Info) << "[CartesianServo] Started. alpha="
                        << smoothing_alpha_
                        << " max_v=" << max_linear_speed_ << " m/s"
                        << " watchdog=" << watchdog_timeout_ms_ << " ms"
                        << RTT::endlog();
    return true;
}

// =============================================================================
// updateHook
// =============================================================================

void CartesianServo::updateHook() {
    // --- 1. Always read FK feedback (keep port fresh) ---
    geometry_msgs::Pose fk_pose;
    port_cartesian_position_.read(fk_pose);

    // --- 2. Read parameter updates ---
    readParameterUpdates();

    // --- 3. Check for new Cartesian command ---
    geometry_msgs::Pose new_cmd;
    if (RTT::NewData == port_cartesian_command_.read(new_cmd)) {
        if (isFinitePose(new_cmd)) {
            target_pose_ = new_cmd;
            last_command_time_ = rtt_rosclock::host_now();

            if (state_ == IDLE) {
                if (debug_) {
                    RTT::log(RTT::Info) << "[CartesianServo] IDLE -> TRACKING"
                                       << RTT::endlog();
                }
            }
            state_ = TRACKING;
        } else {
            RTT::log(RTT::Warning) << "[CartesianServo] Received non-finite pose, ignoring"
                                   << RTT::endlog();
        }
    }

    // --- 4. First iteration: seed differentiation state from IK ---
    if (first_iteration_) {
        Eigen::VectorXd ik_seed;
        if (port_ik_output_.read(ik_seed) == RTT::NewData) {
            prev_ik_output_ = ik_seed;
        }
        first_iteration_ = false;
    }

    // --- 5. Check cancel/stop ---
    std_msgs::Bool cancel;
    if (RTT::NewData == port_cancel_servo_.read(cancel) && cancel.data) {
        if (state_ == TRACKING) {
            state_ = DECELERATING;
            if (debug_) {
                RTT::log(RTT::Info) << "[CartesianServo] Cancel -> DECELERATING"
                                   << RTT::endlog();
            }
        }
    }

    // --- 6. Check IK failure ---
    std_msgs::Bool ik_fail;
    if (RTT::NewData == port_ik_failure_.read(ik_fail) && ik_fail.data) {
        if (state_ == TRACKING) {
            state_ = DECELERATING;
            RTT::log(RTT::Warning) << "[CartesianServo] IK failure -> DECELERATING"
                                   << RTT::endlog();
        }
    }

    // --- 7. Watchdog check ---
    if (state_ == TRACKING) {
        double elapsed_ms = (rtt_rosclock::host_now() - last_command_time_).toSec() * 1000.0;
        if (elapsed_ms > watchdog_timeout_ms_) {
            state_ = DECELERATING;
            if (debug_) {
                RTT::log(RTT::Info) << "[CartesianServo] Watchdog timeout ("
                                   << elapsed_ms << " ms) -> DECELERATING"
                                   << RTT::endlog();
            }
        }
    }

    // --- 8. Exponential smoothing ---
    double alpha;
    switch (state_) {
        case IDLE:
            alpha = 1.0;  // hold position, no change
            break;
        case TRACKING:
            alpha = smoothing_alpha_;
            break;
        case DECELERATING:
            alpha = decel_smoothing_alpha_;
            // freeze target to current output (decelerates naturally)
            target_pose_ = smooth_output_;
            break;
    }

    // position smoothing
    smooth_output_.position.x = alpha * smooth_output_.position.x
                              + (1.0 - alpha) * target_pose_.position.x;
    smooth_output_.position.y = alpha * smooth_output_.position.y
                              + (1.0 - alpha) * target_pose_.position.y;
    smooth_output_.position.z = alpha * smooth_output_.position.z
                              + (1.0 - alpha) * target_pose_.position.z;

    // orientation smoothing (SLERP)
    Eigen::Quaterniond q_target(
        target_pose_.orientation.w,
        target_pose_.orientation.x,
        target_pose_.orientation.y,
        target_pose_.orientation.z);
    q_target.normalize();

    // handle quaternion sign flip (q and -q are the same rotation)
    if (smooth_quat_.dot(q_target) < 0.0) {
        q_target = Eigen::Quaterniond(
            -q_target.w(), -q_target.x(), -q_target.y(), -q_target.z());
    }

    smooth_quat_ = smooth_quat_.slerp(1.0 - alpha, q_target);
    smooth_quat_.normalize();

    smooth_output_.orientation.w = smooth_quat_.w();
    smooth_output_.orientation.x = smooth_quat_.x();
    smooth_output_.orientation.y = smooth_quat_.y();
    smooth_output_.orientation.z = smooth_quat_.z();

    // --- 9. Cartesian velocity estimation (numerical differentiation) ---
    double vx = (smooth_output_.position.x - prev_smooth_output_.position.x) / dt_;
    double vy = (smooth_output_.position.y - prev_smooth_output_.position.y) / dt_;
    double vz = (smooth_output_.position.z - prev_smooth_output_.position.z) / dt_;
    double linear_speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    // angular velocity from quaternion delta
    Eigen::Quaterniond dq = smooth_quat_ * prev_smooth_quat_.inverse();
    dq.normalize();
    Eigen::AngleAxisd aa(dq);
    Eigen::Vector3d omega = aa.axis() * aa.angle() / dt_;
    double angular_speed = omega.norm();

    // --- 10. Velocity limiting ---
    clampLinearVelocity(vx, vy, vz, linear_speed);
    clampAngularVelocity(omega, angular_speed);

    // --- 11. DECELERATING -> IDLE transition ---
    if (state_ == DECELERATING && linear_speed < idle_velocity_threshold_) {
        state_ = IDLE;
        servo_active_msg_.data = false;
        port_servo_active_.write(servo_active_msg_);
        if (debug_) {
            RTT::log(RTT::Info) << "[CartesianServo] DECELERATING -> IDLE"
                               << RTT::endlog();
        }
    }

    // --- 12. Write Cartesian pose output to IK ---
    port_cartesian_position_command_.write(smooth_output_);

    // --- 13. Write Cartesian velocity (monitoring) ---
    cart_vel_msg_.data[0] = vx;
    cart_vel_msg_.data[1] = vy;
    cart_vel_msg_.data[2] = vz;
    cart_vel_msg_.data[3] = omega.x();
    cart_vel_msg_.data[4] = omega.y();
    cart_vel_msg_.data[5] = omega.z();
    port_cartesian_velocity_.write(cart_vel_msg_);

    // --- 14. Torque mode: joint velocity/acceleration ---
    if (torque_mode_) {
        Eigen::VectorXd ik_out(CS_DOF);
        if (port_ik_output_.read(ik_out) == RTT::NewData) {
            Eigen::VectorXd dq_raw  = (ik_out - prev_ik_output_) / dt_;
            Eigen::VectorXd ddq_raw = (dq_raw - prev_dq_) / dt_;

            for (size_t i = 0; i < CS_DOF; i++) {
                dq_filtered_[i]  = vel_filter_alpha_ * dq_filtered_[i]
                                 + (1.0 - vel_filter_alpha_) * dq_raw[i];
                ddq_filtered_[i] = acc_filter_alpha_ * ddq_filtered_[i]
                                 + (1.0 - acc_filter_alpha_) * ddq_raw[i];
            }

            prev_ik_output_ = ik_out;
            prev_dq_        = dq_raw;
        }

        port_command_joint_velocity_.write(dq_filtered_);
        port_command_joint_acceleration_.write(ddq_filtered_);
    }

    // --- 15. Status ---
    int status_code;
    switch (state_) {
        case IDLE:          status_code = 0; break;
        case TRACKING:      status_code = 1; break;
        case DECELERATING:  status_code = 2; break;
    }
    servo_status_msg_.data = status_code;
    port_servo_status_.write(servo_status_msg_);

    if (state_ == TRACKING) {
        servo_active_msg_.data = true;
        port_servo_active_.write(servo_active_msg_);
    }

    // --- 16. Store previous values ---
    prev_smooth_output_ = smooth_output_;
    prev_smooth_quat_   = smooth_quat_;
}

// =============================================================================
// stopHook
// =============================================================================

void CartesianServo::stopHook() {
    state_ = IDLE;
    servo_active_msg_.data = false;
    port_servo_active_.write(servo_active_msg_);
    RTT::log(RTT::Info) << "[CartesianServo] Stopped." << RTT::endlog();
}

// =============================================================================
// Helpers
// =============================================================================

void CartesianServo::readParameterUpdates() {
    std_msgs::Float32MultiArray msg;

    if (RTT::NewData == port_new_smoothing_alpha_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v >= 0.0 && v < 1.0) {
            smoothing_alpha_ = v;
            if (debug_) {
                RTT::log(RTT::Info) << "[CartesianServo] smoothing_alpha = "
                                   << v << RTT::endlog();
            }
        }
    }
    if (RTT::NewData == port_new_max_linear_speed_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v > 0.0) {
            max_linear_speed_ = v;
            if (debug_) {
                RTT::log(RTT::Info) << "[CartesianServo] max_linear_speed = "
                                   << v << RTT::endlog();
            }
        }
    }
    if (RTT::NewData == port_new_max_angular_speed_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v > 0.0) {
            max_angular_speed_ = v;
        }
    }
    if (RTT::NewData == port_new_watchdog_timeout_.read(msg) && msg.data.size() >= 1) {
        double v = msg.data[0];
        if (v > 0.0) {
            watchdog_timeout_ms_ = v;
        }
    }

    std_msgs::Bool bmsg;
    if (RTT::NewData == port_new_torque_mode_.read(bmsg)) {
        torque_mode_ = bmsg.data;
        if (debug_) {
            RTT::log(RTT::Info) << "[CartesianServo] torque_mode = "
                               << (torque_mode_ ? "true" : "false")
                               << RTT::endlog();
        }
    }
    if (RTT::NewData == port_new_debug_.read(bmsg)) {
        debug_ = bmsg.data;
    }
}

bool CartesianServo::isFinitePose(const geometry_msgs::Pose& p) const {
    return std::isfinite(p.position.x) && std::isfinite(p.position.y)
        && std::isfinite(p.position.z)
        && std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y)
        && std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
}

void CartesianServo::clampLinearVelocity(double& vx, double& vy, double& vz,
                                         double& linear_speed) {
    if (linear_speed > max_linear_speed_ && linear_speed > 1e-9) {
        double scale = max_linear_speed_ / linear_speed;
        // scale back the smoothed position to limit velocity
        smooth_output_.position.x = prev_smooth_output_.position.x
            + (smooth_output_.position.x - prev_smooth_output_.position.x) * scale;
        smooth_output_.position.y = prev_smooth_output_.position.y
            + (smooth_output_.position.y - prev_smooth_output_.position.y) * scale;
        smooth_output_.position.z = prev_smooth_output_.position.z
            + (smooth_output_.position.z - prev_smooth_output_.position.z) * scale;

        vx = (smooth_output_.position.x - prev_smooth_output_.position.x) / dt_;
        vy = (smooth_output_.position.y - prev_smooth_output_.position.y) / dt_;
        vz = (smooth_output_.position.z - prev_smooth_output_.position.z) / dt_;
        linear_speed = max_linear_speed_;
    }
}

void CartesianServo::clampAngularVelocity(Eigen::Vector3d& omega,
                                          double& angular_speed) {
    if (angular_speed > max_angular_speed_ && angular_speed > 1e-9) {
        double scale = max_angular_speed_ / angular_speed;
        // scale back orientation toward previous
        smooth_quat_ = prev_smooth_quat_.slerp(scale, smooth_quat_);
        smooth_quat_.normalize();

        smooth_output_.orientation.w = smooth_quat_.w();
        smooth_output_.orientation.x = smooth_quat_.x();
        smooth_output_.orientation.y = smooth_quat_.y();
        smooth_output_.orientation.z = smooth_quat_.z();

        // recompute omega
        Eigen::Quaterniond dq = smooth_quat_ * prev_smooth_quat_.inverse();
        dq.normalize();
        Eigen::AngleAxisd aa(dq);
        omega = aa.axis() * aa.angle() / dt_;
        angular_speed = omega.norm();
    }
}

ORO_CREATE_COMPONENT(CartesianServo)
