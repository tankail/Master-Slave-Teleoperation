#!/usr/bin/env python3
"""
Gravity Compensation Controller Node

This node integrates motor state reading and gravity compensation:
1. Subscribes to /master_controller_manipulator_state to get current joint states
2. Computes gravity compensation torques using Pinocchio
3. Publishes joint states with gravity compensation torques

Author: Auto-generated
Date: 2026-01-07
"""

import rospy
import numpy as np
import pinocchio as pin
import os
from sensor_msgs.msg import JointState
from threading import Lock


class GravityCompensationController:
    """
    ROS node for real-time gravity compensation
    """

    # Dual-arm joint names (must match URDF and motor order)
    ARM_JOINT_NAMES = [
        'l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
        'l_upper_arm_joint', 'l_elbow_joint',
        'l_wrist_joint', 'l_gripper_joint',
        'r_shoulder_pitch_joint', 'r_shoulder_roll_joint',
        'r_upper_arm_joint', 'r_elbow_joint',
        'r_wrist_joint', 'r_gripper_joint',
    ]

    def __init__(self):
        """Initialize the gravity compensation controller"""
        rospy.init_node('gravity_compensation_controller', anonymous=False)

        # Parameters
        self.publish_rate = rospy.get_param('~publish_rate', 100)  # Hz
        self.enable_compensation = rospy.get_param('~enable_compensation', True)
        self.compensation_scale = rospy.get_param('~compensation_scale', 1.0)  # Scale factor for tuning

        # URDF path
        urdf_path = rospy.get_param('~urdf_path', self._get_default_urdf_path())
        rospy.loginfo(f"Loading URDF from: {urdf_path}")

        # Load Pinocchio model
        if not os.path.exists(urdf_path):
            rospy.logerr(f"URDF file not found: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        rospy.loginfo(f"Pinocchio model loaded: {self.model.nq} DOFs, {self.model.njoints} joints")

        # Build joint mapping
        self._build_joint_mapping()


        # State variables
        self.current_joint_state = None
        self.state_lock = Lock()

        # ROS Publishers and Subscribers
        self.joint_state_sub = rospy.Subscriber(
            '/master_controller_manipulator_state',
            JointState,
            self.joint_state_callback,
            queue_size=10
        )

        self.gravity_comp_pub = rospy.Publisher(
            '/gravity_compensation_torques',
            JointState,
            queue_size=10
        )

        # Statistics
        self.msg_count = 0
        self.error_count = 0

        rospy.loginfo("\033[1;32mGravity Compensation Controller Started\033[0m")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
        rospy.loginfo(f"Compensation enabled: {self.enable_compensation}")
        rospy.loginfo(f"Compensation scale: {self.compensation_scale}")

    def _get_default_urdf_path(self):
        """Get default URDF path relative to this script"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(
            script_dir,
            "../model/pi_plus_arm6_head5_29/urdf/pi_plus_arm6_head5_29.urdf"
        )
        return os.path.normpath(urdf_path)

    def _build_joint_mapping(self):
        """Build mapping from joint names to configuration indices"""
        self.joint_name_to_id = {}
        self.arm_joint_indices = []

        # Create name to ID mapping
        for joint_id in range(self.model.njoints):
            joint_name = self.model.names[joint_id]
            self.joint_name_to_id[joint_name] = joint_id

        # Find configuration indices for arm joints
        for joint_name in self.ARM_JOINT_NAMES:
            if joint_name in self.joint_name_to_id:
                joint_id = self.joint_name_to_id[joint_name]
                idx_q = self.model.joints[joint_id].idx_q
                print(f"{joint_name}: {idx_q}")
                self.arm_joint_indices.append(idx_q)
            else:
                rospy.logwarn(f"Joint not found in URDF: {joint_name}")
                self.arm_joint_indices.append(-1)  # Mark as missing

        rospy.loginfo(f"Mapped {len([i for i in self.arm_joint_indices if i >= 0])} arm joints")

    def joint_state_callback(self, msg):
        """Callback for receiving joint states from master controller"""
        with self.state_lock:
            self.current_joint_state = msg

    def compute_gravity_compensation(self, joint_positions):
        """
        Compute gravity compensation torques for given joint positions

        Args:
            joint_positions: Array of joint positions (only arm joints)

        Returns:
            Array of gravity compensation torques for arm joints
        """
        # Create full configuration vector (initialize to zero)
        q = np.zeros(self.model.nq)

        # joint_positions = np.zeros(len(joint_positions_))
        # joint_positions[0:6] = joint_positions_[6:12]
        # joint_positions[6:12] = joint_positions_[0:6]

        # Fill in arm joint positions
        for i, idx_q in enumerate(self.arm_joint_indices):
            if idx_q >= 0 and i < len(joint_positions):
                q[idx_q] = joint_positions[i]
                if(i == 0):
                    q[idx_q] = q[idx_q] - np.pi / 2
                elif(i == 1):
                    q[idx_q] = q[idx_q] - np.pi / 2
                elif(i == 6):
                    q[idx_q] = q[idx_q] + np.pi / 2
                elif(i == 7):
                    q[idx_q] = q[idx_q] + np.pi / 2
        
        print("q: ", q)

        # Compute gravity torques using Pinocchio
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q)

        # Extract torques for arm joints
        arm_torques = np.array([
            tau_g[idx_q] if idx_q >= 0 else 0.0
            for idx_q in self.arm_joint_indices
        ])

        # Apply scale factor
        arm_torques *= self.compensation_scale
        # arm_torques_ = np.zeros(len(joint_positions_))
        # arm_torques_[6:12] = arm_torques[0:6]
        # arm_torques_[0:6] = arm_torques[6:12]

        return arm_torques

    def run(self):
        """Main control loop"""
        rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("Waiting for first joint state message...")

        # Wait for first message
        while not rospy.is_shutdown() and self.current_joint_state is None:
            rospy.sleep(0.1)

        rospy.loginfo("Received first joint state, starting gravity compensation")

        while not rospy.is_shutdown():
            try:
                # Get current state (thread-safe)
                with self.state_lock:
                    if self.current_joint_state is None:
                        rate.sleep()
                        continue

                    joint_state = self.current_joint_state

                # Compute gravity compensation torques
                if self.enable_compensation and len(joint_state.position) >= len(self.ARM_JOINT_NAMES):
                    # Use first N positions as arm joint positions
                    arm_positions = joint_state.position[:len(self.ARM_JOINT_NAMES)]
                    gravity_torques = self.compute_gravity_compensation(arm_positions)
                else:
                    # Compensation disabled or insufficient position data
                    gravity_torques = np.zeros(len(self.ARM_JOINT_NAMES))

                # Create output message
                output_msg = JointState()
                output_msg.header.stamp = rospy.Time.now()
                output_msg.name = self.ARM_JOINT_NAMES

                # Copy positions and velocities from input
                if len(joint_state.position) >= len(self.ARM_JOINT_NAMES):
                    output_msg.position = list(joint_state.position[:len(self.ARM_JOINT_NAMES)])
                else:
                    output_msg.position = [0.0] * len(self.ARM_JOINT_NAMES)

                if len(joint_state.velocity) >= len(self.ARM_JOINT_NAMES):
                    output_msg.velocity = list(joint_state.velocity[:len(self.ARM_JOINT_NAMES)])
                else:
                    output_msg.velocity = [0.0] * len(self.ARM_JOINT_NAMES)

                # Set effort to gravity compensation torques
                output_msg.effort = gravity_torques.tolist()

                # Publish
                self.gravity_comp_pub.publish(output_msg)

                # Statistics
                self.msg_count += 1
                if self.msg_count % 1000 == 0:
                    rospy.loginfo(f"Published {self.msg_count} messages, {self.error_count} errors")
                    # Print sample torques
                    rospy.loginfo(f"Sample gravity torques: {gravity_torques[:3]}")

            except Exception as e:
                self.error_count += 1
                rospy.logerr(f"Error in control loop: {e}")
                if self.error_count % 100 == 1:  # Log first error and every 100th
                    import traceback
                    rospy.logerr(traceback.format_exc())

            rate.sleep()

    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down gravity compensation controller")
        rospy.loginfo(f"Total messages: {self.msg_count}, Errors: {self.error_count}")


def main():
    """Main entry point"""
    try:
        controller = GravityCompensationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
        return 1

    return 0


if __name__ == '__main__':
    exit(main())
