#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Haptic Bridge — Clutched Incremental Teleoperation

Phantom Omni/Touch  -->  CartesianServo  -->  IK  -->  Robot

Tryb dzialania (clutched incremental):
  1. Przycisk wcisniety  -> zapamietaj pozycje joysticka (anchor_haptic)
                            i robota (anchor_robot)
  2. Podczas trzymania   -> delta = (haptic - anchor_haptic) * scale
                            target_robot = anchor_robot + R_align * delta
                            publikuj na /es_cartesian_servo/command
  3. Przycisk zwolniony  -> stop publikacji, CartesianServo watchdog
                            zatrzymuje robota plynnie

Parametry ROS (konfiguracja w launch file):
  ~haptic_pose_topic    - topic z PoseStamped haptic device
  ~haptic_button_topic  - topic z Bool (true=wcisniety)
  ~robot_pose_topic     - topic z aktualny TCP z FK
  ~servo_command_topic  - topic wyjsciowy do CartesianServo
  ~scale_position       - skalowanie pozycji (domyslnie 3.0)
  ~scale_orientation    - skalowanie orientacji 0.0-1.0 (1.0=pelne)
  ~frame_rotation_rpy   - [roll, pitch, yaw] wyrownanie ukladow [rad]
  ~dead_zone            - strefa martwa pozycji [m w przestrzeni haptic]
  ~publish_rate         - czestotliwosc publikacji [Hz]
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float32MultiArray
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    quaternion_multiply,
    quaternion_inverse,
    euler_matrix,
)


class HapticBridge(object):
    def __init__(self):
        rospy.init_node("haptic_bridge", anonymous=False)

        # --- parameters ---
        self.scale_pos = rospy.get_param("~scale_position", 3.0)
        self.scale_ori = rospy.get_param("~scale_orientation", 1.0)
        self.dead_zone = rospy.get_param("~dead_zone", 0.002)  # 2mm in haptic space
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)  # Hz

        # frame alignment: rotation from haptic frame to robot base frame
        # Phantom Omni default: X-right, Y-up, Z-toward-user
        # Robot typical:        X-forward, Y-left, Z-up
        # Default RPY rotates haptic -> robot (tune per setup)
        rpy = rospy.get_param("~frame_rotation_rpy", [0.0, 0.0, 0.0])
        self.R_align = euler_matrix(rpy[0], rpy[1], rpy[2])[:3, :3]

        # --- topics ---
        haptic_pose_topic = rospy.get_param("~haptic_pose_topic", "/phantom/pose")
        haptic_button_topic = rospy.get_param("~haptic_button_topic", "/phantom/button")
        robot_pose_topic = rospy.get_param("~robot_pose_topic", "/es_arm/cartesian_pose")
        servo_command_topic = rospy.get_param("~servo_command_topic", "/es_cartesian_servo/command")
        servo_status_topic = rospy.get_param("~servo_status_topic", "/es_cartesian_servo/active")

        # --- state ---
        self.engaged = False
        self.anchor_haptic_pos = None    # np.array([x,y,z])
        self.anchor_haptic_quat = None   # np.array([x,y,z,w])
        self.anchor_robot_pos = None
        self.anchor_robot_quat = None

        self.haptic_pos = None
        self.haptic_quat = None
        self.robot_pos = None
        self.robot_quat = None
        self.button_pressed = False

        self.haptic_data_received = False
        self.robot_data_received = False

        # --- publishers ---
        self.pub_command = rospy.Publisher(servo_command_topic, Pose, queue_size=1)
        self.pub_status = rospy.Publisher("~bridge_status", Bool, queue_size=1)

        # --- subscribers ---
        rospy.Subscriber(haptic_pose_topic, PoseStamped, self._cb_haptic_pose, queue_size=1)
        rospy.Subscriber(haptic_button_topic, Bool, self._cb_button, queue_size=1)
        rospy.Subscriber(robot_pose_topic, Pose, self._cb_robot_pose, queue_size=1)

        rospy.loginfo("[HapticBridge] Initialized. scale_pos=%.1f, scale_ori=%.1f, "
                      "dead_zone=%.4f m, rate=%.0f Hz",
                      self.scale_pos, self.scale_ori, self.dead_zone, self.publish_rate)
        rospy.loginfo("[HapticBridge] Haptic: %s  Button: %s  Robot FK: %s",
                      haptic_pose_topic, haptic_button_topic, robot_pose_topic)
        rospy.loginfo("[HapticBridge] Output: %s", servo_command_topic)
        rospy.loginfo("[HapticBridge] Frame rotation RPY: %s", rpy)

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cb_haptic_pose(self, msg):
        """PoseStamped from haptic device driver."""
        p = msg.pose.position
        o = msg.pose.orientation
        self.haptic_pos = np.array([p.x, p.y, p.z])
        self.haptic_quat = np.array([o.x, o.y, o.z, o.w])
        self.haptic_data_received = True

    def _cb_button(self, msg):
        """Bool — clutch button state."""
        new_state = msg.data
        if new_state and not self.button_pressed:
            self._on_engage()
        elif not new_state and self.button_pressed:
            self._on_disengage()
        self.button_pressed = new_state

    def _cb_robot_pose(self, msg):
        """Current TCP pose from FK (geometry_msgs/Pose)."""
        p = msg.position
        o = msg.orientation
        self.robot_pos = np.array([p.x, p.y, p.z])
        self.robot_quat = np.array([o.x, o.y, o.z, o.w])
        self.robot_data_received = True

    # =========================================================================
    # Engage / Disengage
    # =========================================================================

    def _on_engage(self):
        """Button pressed — record anchor poses."""
        if self.haptic_pos is None or self.robot_pos is None:
            rospy.logwarn("[HapticBridge] Cannot engage: no haptic or robot data yet")
            return

        self.anchor_haptic_pos = self.haptic_pos.copy()
        self.anchor_haptic_quat = self.haptic_quat.copy()
        self.anchor_robot_pos = self.robot_pos.copy()
        self.anchor_robot_quat = self.robot_quat.copy()
        self.engaged = True

        rospy.loginfo("[HapticBridge] ENGAGED at robot=[%.3f, %.3f, %.3f]",
                      self.anchor_robot_pos[0],
                      self.anchor_robot_pos[1],
                      self.anchor_robot_pos[2])

        status = Bool()
        status.data = True
        self.pub_status.publish(status)

    def _on_disengage(self):
        """Button released — stop commanding."""
        self.engaged = False
        rospy.loginfo("[HapticBridge] DISENGAGED (watchdog will stop robot)")

        status = Bool()
        status.data = False
        self.pub_status.publish(status)

    # =========================================================================
    # Compute target pose
    # =========================================================================

    def _compute_target(self):
        """Compute robot target from haptic delta + anchor."""
        # position delta in haptic frame
        delta_haptic = self.haptic_pos - self.anchor_haptic_pos

        # dead zone — ignore small noise
        delta_norm = np.linalg.norm(delta_haptic)
        if delta_norm < self.dead_zone:
            delta_haptic = np.zeros(3)
        else:
            # subtract dead zone from magnitude, keep direction
            delta_haptic = delta_haptic * (delta_norm - self.dead_zone) / delta_norm

        # rotate to robot frame and scale
        delta_robot = self.R_align.dot(delta_haptic) * self.scale_pos

        # target position = anchor + delta
        target_pos = self.anchor_robot_pos + delta_robot

        # orientation: incremental rotation from haptic
        if self.scale_ori > 0.01:
            target_quat = self._compute_orientation_delta()
        else:
            target_quat = self.anchor_robot_quat.copy()

        return target_pos, target_quat

    def _compute_orientation_delta(self):
        """
        Compute target orientation using incremental quaternion delta.

        delta_q_haptic = q_haptic_current * q_haptic_anchor^-1
        (this is the rotation the user made since engaging)

        Then apply (scaled) to robot anchor orientation:
        target_q_robot = delta_q_aligned * anchor_q_robot
        """
        # haptic rotation delta
        q_anchor_inv = quaternion_inverse(self.anchor_haptic_quat)
        delta_q_haptic = quaternion_multiply(self.haptic_quat, q_anchor_inv)

        # rotate delta into robot frame using R_align
        R_delta = quaternion_matrix(delta_q_haptic)[:3, :3]
        R_delta_aligned = self.R_align.dot(R_delta).dot(self.R_align.T)

        # optional: scale the rotation (interpolate between identity and delta)
        if self.scale_ori < 0.99:
            # SLERP between identity and R_delta_aligned
            # approximate: axis-angle scaling
            from tf.transformations import rotation_from_matrix, rotation_matrix
            try:
                angle, direction, _ = rotation_from_matrix(
                    np.vstack([np.hstack([R_delta_aligned, [[0], [0], [0]]]),
                               [0, 0, 0, 1]]))
                angle_scaled = angle * self.scale_ori
                R_scaled = rotation_matrix(angle_scaled, direction)[:3, :3]
            except ValueError:
                R_scaled = R_delta_aligned
        else:
            R_scaled = R_delta_aligned

        # build 4x4 for quaternion extraction
        M = np.eye(4)
        M[:3, :3] = R_scaled
        delta_q_aligned = quaternion_from_matrix(M)  # [x,y,z,w] from tf

        # apply to robot anchor: target = delta * anchor
        target_q = quaternion_multiply(delta_q_aligned, self.anchor_robot_quat)

        # normalize
        target_q = target_q / np.linalg.norm(target_q)
        return target_q

    # =========================================================================
    # Main loop
    # =========================================================================

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("[HapticBridge] Waiting for haptic and robot data...")
        while not rospy.is_shutdown():
            if self.haptic_data_received and self.robot_data_received:
                break
            rate.sleep()
        rospy.loginfo("[HapticBridge] Data received. Ready. Press button to engage.")

        while not rospy.is_shutdown():
            if self.engaged and self.haptic_pos is not None:
                target_pos, target_quat = self._compute_target()

                cmd = Pose()
                cmd.position.x = target_pos[0]
                cmd.position.y = target_pos[1]
                cmd.position.z = target_pos[2]
                cmd.orientation.x = target_quat[0]
                cmd.orientation.y = target_quat[1]
                cmd.orientation.z = target_quat[2]
                cmd.orientation.w = target_quat[3]

                self.pub_command.publish(cmd)

            rate.sleep()


def main():
    try:
        bridge = HapticBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
