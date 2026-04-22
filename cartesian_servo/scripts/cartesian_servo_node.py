#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Cartesian Servo Node — Python replacement for OROCOS CartesianServo component.

Combines haptic bridge (clutched incremental) + exponential smoothing
in a single ROS node. Publishes directly to IK input via ROS topic.

No OROCOS component needed = no semaphore allocation = no memory issues.

Architecture:
  /phantom/pose (PoseStamped, ~1kHz)
       |
  [clutch + scale + frame rotate]
       |
  [exponential smoothing @ publish_rate Hz]
       |
  /es_cartesian_servo/command (Pose)
       |
  stream() -> IK.InputEndEffectorPose (in .ops)
       |
  IK -> LimitDetector -> ESJ2M -> Motors (existing OROCOS pipeline)
"""

import rospy
import numpy as np
import threading
import math
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float32MultiArray, Int32


# ============================================================================
# Quaternion math (pure numpy, no tf dependency)
# Quaternion format: [x, y, z, w]
# ============================================================================

def quat_multiply(q1, q2):
    """Hamilton product, format [x,y,z,w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])

def quat_inverse(q):
    """Inverse (conjugate for unit quaternion), format [x,y,z,w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])

def quat_to_matrix(q):
    """Quaternion [x,y,z,w] -> 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])

def matrix_to_quat(R):
    """3x3 rotation matrix -> quaternion [x,y,z,w]."""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])

def euler_to_matrix(roll, pitch, yaw):
    """RPY -> 3x3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,           cp*cr],
    ])


class CartesianServoNode(object):

    # states
    IDLE = 0
    TRACKING = 1
    DECELERATING = 2

    def __init__(self):
        rospy.init_node("cartesian_servo", anonymous=False)

        # --- parameters ---
        self.scale_pos = rospy.get_param("~scale_position", 3.0)
        self.scale_ori = rospy.get_param("~scale_orientation", 1.0)
        self.dead_zone = rospy.get_param("~dead_zone", 0.002)
        self.publish_rate = rospy.get_param("~publish_rate", 250.0)
        self.smoothing_alpha = rospy.get_param("~smoothing_alpha", 0.85)
        self.decel_alpha = rospy.get_param("~decel_smoothing_alpha", 0.98)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.watchdog_timeout_ms = rospy.get_param("~watchdog_timeout_ms", 200.0)
        self.idle_vel_threshold = rospy.get_param("~idle_velocity_threshold", 0.001)

        rpy = rospy.get_param("~frame_rotation_rpy", [0.0, 0.0, 0.0])
        self.R_align = euler_to_matrix(rpy[0], rpy[1], rpy[2])

        # --- topics ---
        haptic_pose_topic = rospy.get_param("~haptic_pose_topic", "/phantom/pose")
        haptic_button_topic = rospy.get_param("~haptic_button_topic", "/phantom/button")
        robot_pose_topic = rospy.get_param("~robot_pose_topic", "/es_arm/cartesian_pose")
        servo_command_topic = rospy.get_param("~servo_command_topic",
                                              "/es_cartesian_servo/command")

        # --- state (thread-safe via lock) ---
        self.lock = threading.Lock()
        self.state = self.IDLE

        # haptic
        self.haptic_pos = None
        self.haptic_quat = None
        self.button_pressed = False

        # anchors
        self.anchor_haptic_pos = None
        self.anchor_haptic_quat = None
        self.anchor_robot_pos = None
        self.anchor_robot_quat = None

        # robot FK feedback
        self.robot_pos = None
        self.robot_quat = None

        # smoothed output (initialized from FK)
        self.smooth_pos = None
        self.smooth_quat = None
        self.prev_smooth_pos = None
        self.prev_smooth_quat = None
        self.target_pos = None
        self.target_quat = None

        self.last_command_time = rospy.Time.now()
        self.engaged = False
        self.initialized = False

        self.dt = 1.0 / self.publish_rate

        # --- publishers ---
        self.pub_command = rospy.Publisher(servo_command_topic, Pose, queue_size=1)
        self.pub_velocity = rospy.Publisher("~velocity", Float32MultiArray, queue_size=1)
        self.pub_active = rospy.Publisher("~active", Bool, queue_size=1)
        self.pub_status = rospy.Publisher("~status", Int32, queue_size=1)

        # --- subscribers ---
        rospy.Subscriber(haptic_pose_topic, PoseStamped, self._cb_haptic, queue_size=1)
        rospy.Subscriber(haptic_button_topic, Bool, self._cb_button, queue_size=1)
        rospy.Subscriber(robot_pose_topic, Pose, self._cb_robot, queue_size=1)

        # --- online parameter tuning ---
        rospy.Subscriber("~smoothing_alpha", Float32MultiArray,
                         self._cb_param_alpha, queue_size=1)
        rospy.Subscriber("~max_linear_speed", Float32MultiArray,
                         self._cb_param_speed, queue_size=1)

        rospy.loginfo("[CartesianServo] Started. rate=%.0f Hz, alpha=%.2f, "
                      "scale=%.1f, max_v=%.2f m/s",
                      self.publish_rate, self.smoothing_alpha,
                      self.scale_pos, self.max_linear_speed)
        rospy.loginfo("[CartesianServo] Haptic: %s  Button: %s  FK: %s  Out: %s",
                      haptic_pose_topic, haptic_button_topic,
                      robot_pose_topic, servo_command_topic)

    # =========================================================================
    # Callbacks (run in subscriber threads)
    # =========================================================================

    def _cb_haptic(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        with self.lock:
            self.haptic_pos = np.array([p.x, p.y, p.z])
            self.haptic_quat = np.array([o.x, o.y, o.z, o.w])

    def _cb_button(self, msg):
        with self.lock:
            new_state = msg.data
            if new_state and not self.button_pressed:
                self._on_engage()
            elif not new_state and self.button_pressed:
                self._on_disengage()
            self.button_pressed = new_state

    def _cb_robot(self, msg):
        p = msg.position
        o = msg.orientation
        with self.lock:
            self.robot_pos = np.array([p.x, p.y, p.z])
            self.robot_quat = np.array([o.x, o.y, o.z, o.w])
            if not self.initialized:
                self.smooth_pos = self.robot_pos.copy()
                self.smooth_quat = self.robot_quat.copy()
                self.prev_smooth_pos = self.robot_pos.copy()
                self.prev_smooth_quat = self.robot_quat.copy()
                self.target_pos = self.robot_pos.copy()
                self.target_quat = self.robot_quat.copy()
                self.initialized = True
                rospy.loginfo("[CartesianServo] FK init at [%.3f, %.3f, %.3f]",
                              self.robot_pos[0], self.robot_pos[1], self.robot_pos[2])

    def _cb_param_alpha(self, msg):
        if msg.data and 0.0 <= msg.data[0] < 1.0:
            self.smoothing_alpha = msg.data[0]
            rospy.loginfo("[CartesianServo] smoothing_alpha = %.3f", msg.data[0])

    def _cb_param_speed(self, msg):
        if msg.data and msg.data[0] > 0:
            self.max_linear_speed = msg.data[0]
            rospy.loginfo("[CartesianServo] max_linear_speed = %.3f", msg.data[0])

    # =========================================================================
    # Engage / Disengage (called inside lock)
    # =========================================================================

    def _on_engage(self):
        if self.haptic_pos is None or self.robot_pos is None:
            rospy.logwarn("[CartesianServo] Cannot engage: no haptic or robot data")
            return
        self.anchor_haptic_pos = self.haptic_pos.copy()
        self.anchor_haptic_quat = self.haptic_quat.copy()
        self.anchor_robot_pos = self.robot_pos.copy()
        self.anchor_robot_quat = self.robot_quat.copy()
        self.engaged = True
        self.state = self.TRACKING
        self.last_command_time = rospy.Time.now()
        rospy.loginfo("[CartesianServo] ENGAGED at [%.3f, %.3f, %.3f]",
                      self.anchor_robot_pos[0], self.anchor_robot_pos[1],
                      self.anchor_robot_pos[2])

    def _on_disengage(self):
        self.engaged = False
        if self.state == self.TRACKING:
            self.state = self.DECELERATING
        rospy.loginfo("[CartesianServo] DISENGAGED -> DECELERATING")

    # =========================================================================
    # Compute target from haptic delta (called inside lock)
    # =========================================================================

    def _compute_target(self):
        if self.anchor_haptic_pos is None:
            return

        # position delta
        delta = self.haptic_pos - self.anchor_haptic_pos
        dnorm = np.linalg.norm(delta)
        if dnorm < self.dead_zone:
            delta = np.zeros(3)
        else:
            delta = delta * (dnorm - self.dead_zone) / dnorm

        delta_robot = self.R_align.dot(delta) * self.scale_pos
        self.target_pos = self.anchor_robot_pos + delta_robot

        # orientation delta
        if self.scale_ori > 0.01:
            q_anchor_inv = quat_inverse(self.anchor_haptic_quat)
            delta_q = quat_multiply(self.haptic_quat, q_anchor_inv)
            R_delta = quat_to_matrix(delta_q)
            R_aligned = self.R_align.dot(R_delta).dot(self.R_align.T)
            delta_q_aligned = matrix_to_quat(R_aligned)
            self.target_quat = quat_multiply(delta_q_aligned,
                                             self.anchor_robot_quat)
            norm = np.linalg.norm(self.target_quat)
            if norm > 1e-10:
                self.target_quat /= norm
        else:
            self.target_quat = self.anchor_robot_quat.copy()

    # =========================================================================
    # Main loop
    # =========================================================================

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("[CartesianServo] Waiting for FK data...")
        while not rospy.is_shutdown() and not self.initialized:
            rate.sleep()
        rospy.loginfo("[CartesianServo] Ready. Press button to engage.")

        cycle = 0
        while not rospy.is_shutdown():
            cycle += 1

            with self.lock:
                # --- update target from haptic ---
                if self.engaged and self.haptic_pos is not None:
                    self._compute_target()
                    self.last_command_time = rospy.Time.now()

                # --- watchdog ---
                if self.state == self.TRACKING:
                    elapsed = (rospy.Time.now() - self.last_command_time).to_sec() * 1000
                    if elapsed > self.watchdog_timeout_ms:
                        self.state = self.DECELERATING

                # --- smoothing ---
                if self.state == self.IDLE:
                    alpha = 1.0
                elif self.state == self.TRACKING:
                    alpha = self.smoothing_alpha
                else:  # DECELERATING
                    alpha = self.decel_alpha
                    self.target_pos = self.smooth_pos.copy()
                    self.target_quat = self.smooth_quat.copy()

                # position
                self.smooth_pos = alpha * self.smooth_pos \
                                + (1.0 - alpha) * self.target_pos

                # orientation SLERP
                q_s = self.smooth_quat
                q_t = self.target_quat
                if np.dot(q_s, q_t) < 0:
                    q_t = -q_t
                t_factor = 1.0 - alpha
                # simple SLERP
                dot = np.clip(np.dot(q_s, q_t), -1.0, 1.0)
                if abs(dot) > 0.9999:
                    self.smooth_quat = q_s + t_factor * (q_t - q_s)
                else:
                    omega = np.arccos(abs(dot))
                    so = np.sin(omega)
                    self.smooth_quat = (np.sin((1.0 - t_factor) * omega) / so) * q_s \
                                     + (np.sin(t_factor * omega) / so) * q_t
                norm = np.linalg.norm(self.smooth_quat)
                if norm > 1e-10:
                    self.smooth_quat /= norm

                # --- velocity ---
                vel = (self.smooth_pos - self.prev_smooth_pos) / self.dt
                speed = np.linalg.norm(vel)

                # --- velocity clamping ---
                if speed > self.max_linear_speed and speed > 1e-9:
                    scale = self.max_linear_speed / speed
                    self.smooth_pos = self.prev_smooth_pos \
                        + (self.smooth_pos - self.prev_smooth_pos) * scale
                    vel = (self.smooth_pos - self.prev_smooth_pos) / self.dt
                    speed = self.max_linear_speed

                # --- DECEL -> IDLE ---
                if self.state == self.DECELERATING and speed < self.idle_vel_threshold:
                    self.state = self.IDLE

                # snapshot for publishing (avoid holding lock during publish)
                out_pos = self.smooth_pos.copy()
                out_quat = self.smooth_quat.copy()
                cur_state = self.state
                cur_vel = vel.copy()
                cur_speed = speed

                self.prev_smooth_pos = self.smooth_pos.copy()
                self.prev_smooth_quat = self.smooth_quat.copy()

            # --- publish (outside lock) ---
            cmd = Pose()
            cmd.position.x = out_pos[0]
            cmd.position.y = out_pos[1]
            cmd.position.z = out_pos[2]
            cmd.orientation.x = out_quat[0]
            cmd.orientation.y = out_quat[1]
            cmd.orientation.z = out_quat[2]
            cmd.orientation.w = out_quat[3]
            self.pub_command.publish(cmd)

            # monitoring (every 10th cycle to reduce overhead)
            if cycle % 10 == 0:
                vel_msg = Float32MultiArray()
                vel_msg.data = [cur_vel[0], cur_vel[1], cur_vel[2], 0, 0, 0]
                self.pub_velocity.publish(vel_msg)

                self.pub_status.publish(Int32(data=cur_state))
                self.pub_active.publish(Bool(data=(cur_state == self.TRACKING)))

            # alive log
            if cycle % int(self.publish_rate * 5) == 0:
                rospy.loginfo("[CartesianServo] cycle=%d state=%d pos=[%.3f,%.3f,%.3f] "
                              "v=%.4f m/s",
                              cycle, cur_state, out_pos[0], out_pos[1], out_pos[2],
                              cur_speed)

            rate.sleep()


def main():
    try:
        node = CartesianServoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
