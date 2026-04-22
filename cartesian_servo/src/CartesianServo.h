#ifndef CARTESIAN_SERVO_H
#define CARTESIAN_SERVO_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <rtt_rosclock/rtt_rosclock.h>

static const size_t CS_DOF = 6;

class CartesianServo : public RTT::TaskContext {
public:
    explicit CartesianServo(const std::string& name);
    ~CartesianServo();

    bool configureHook() override;
    bool startHook()     override;
    void updateHook()    override;
    void stopHook()      override;

private:
    enum ServoState { IDLE, TRACKING, DECELERATING };
    ServoState state_;

    // --- Input ports ---
    // CartesianPosition is EventPort: FK writes at 500Hz -> triggers updateHook
    RTT::InputPort<geometry_msgs::Pose>              port_cartesian_position_;
    RTT::InputPort<geometry_msgs::Pose>              port_cartesian_command_;
    RTT::InputPort<Eigen::VectorXd>                  port_joint_position_;
    RTT::InputPort<Eigen::VectorXd>                  port_ik_output_;
    RTT::InputPort<std_msgs::Bool>                   port_ik_failure_;
    RTT::InputPort<std_msgs::Int32>                  port_cancel_servo_;

    RTT::InputPort<std_msgs::Float32MultiArray>      port_new_smoothing_alpha_;
    RTT::InputPort<std_msgs::Float32MultiArray>      port_new_max_linear_speed_;
    RTT::InputPort<std_msgs::Float32MultiArray>      port_new_max_angular_speed_;
    RTT::InputPort<std_msgs::Float32MultiArray>      port_new_watchdog_timeout_;
    RTT::InputPort<std_msgs::Bool>                   port_new_torque_mode_;
    RTT::InputPort<std_msgs::Bool>                   port_new_debug_;

    // --- Output ports ---
    RTT::OutputPort<geometry_msgs::Pose>             port_cartesian_position_command_;
    RTT::OutputPort<std_msgs::Float32MultiArray>     port_cartesian_velocity_;
    RTT::OutputPort<Eigen::VectorXd>                 port_command_joint_velocity_;
    RTT::OutputPort<Eigen::VectorXd>                 port_command_joint_acceleration_;
    RTT::OutputPort<std_msgs::Bool>                  port_servo_active_;
    RTT::OutputPort<std_msgs::Int32>                 port_servo_status_;

    // --- Properties ---
    double smoothing_alpha_;
    double decel_smoothing_alpha_;
    double watchdog_timeout_ms_;
    double max_linear_speed_;
    double max_angular_speed_;
    double vel_filter_alpha_;
    double acc_filter_alpha_;
    double idle_velocity_threshold_;
    double dt_;
    bool   torque_mode_;
    bool   debug_;

    // --- Internal state ---
    geometry_msgs::Pose target_pose_;
    geometry_msgs::Pose smooth_output_;
    geometry_msgs::Pose prev_smooth_output_;
    Eigen::Quaterniond  smooth_quat_;
    Eigen::Quaterniond  prev_smooth_quat_;

    Eigen::VectorXd prev_ik_output_;
    Eigen::VectorXd prev_dq_;
    Eigen::VectorXd dq_filtered_;
    Eigen::VectorXd ddq_filtered_;

    ros::Time last_command_time_;
    bool      first_iteration_;
    bool      first_fk_received_;
    unsigned long update_count_;

    // output buffers
    std_msgs::Float32MultiArray cart_vel_msg_;
    std_msgs::Bool              servo_active_msg_;
    std_msgs::Int32             servo_status_msg_;

    // --- Helpers ---
    void readParameterUpdates();
    bool isFinitePose(const geometry_msgs::Pose& p) const;
    bool isFiniteQuat(const Eigen::Quaterniond& q) const;
};

#endif
