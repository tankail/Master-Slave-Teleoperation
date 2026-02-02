#!/usr/bin/env python3
"""
Standalone Gravity Compensation Test Program (No ROS dependency)

This script uses Pinocchio to compute gravity compensation torques directly.
It can verify:
1. Pinocchio is correctly installed
2. URDF model loads properly
3. Gravity compensation calculation works correctly
"""

import numpy as np
import pinocchio as pin
import os
import sys

# Optional: use matplotlib for plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("[Warning] matplotlib not found, plotting disabled")


class StandaloneGravityCompensator:
    """Standalone gravity compensation calculator"""

    # Dual-arm joint names
    ARM_JOINT_NAMES = [
        'r_shoulder_pitch_joint', 'r_shoulder_roll_joint',
        'r_upper_arm_joint', 'r_elbow_joint',
        'r_wrist_joint', 'r_gripper_joint',
        'l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
        'l_upper_arm_joint', 'l_elbow_joint',
        'l_wrist_joint', 'l_gripper_joint'
    ]

    def __init__(self, urdf_path):
        """
        Initialize Pinocchio model

        Args:
            urdf_path: Path to URDF file
        """
        print(f"[Info] Loading URDF from: {urdf_path}")

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        # Load URDF model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        print(f"[Info] Model loaded successfully!")
        print(f"[Info] Total joints: {self.model.njoints}")
        print(f"[Info] Total DOFs: {self.model.nq}")
        print(f"[Info] Joint names: {[self.model.names[i] for i in range(min(5, self.model.njoints))]}")

        # Build joint index mapping
        self._build_joint_mapping()

    def _build_joint_mapping(self):
        """Build mapping from joint names to indices"""
        self.joint_name_to_id = {}
        self.arm_joint_indices = []

        # Create name to ID mapping for all joints
        for joint_id in range(self.model.njoints):
            joint_name = self.model.names[joint_id]
            self.joint_name_to_id[joint_name] = joint_id

        # Find configuration indices for arm joints
        for joint_name in self.ARM_JOINT_NAMES:
            if joint_name in self.joint_name_to_id:
                joint_id = self.joint_name_to_id[joint_name]
                idx_q = self.model.joints[joint_id].idx_q
                self.arm_joint_indices.append(idx_q)
            else:
                print(f"[Warning] Joint not found: {joint_name}")

        print(f"[Info] Found {len(self.arm_joint_indices)} arm joints")
        print(f"[Info] Arm joint indices: {self.arm_joint_indices}")

    def compute_gravity_torques(self, q):
        """
        Compute gravity compensation torques

        Args:
            q: Configuration vector (all joint positions)

        Returns:
            All joint gravity torques, Arm joint gravity torques
        """
        q = np.array(q, dtype=np.float64).reshape(-1)

        if q.shape[0] != self.model.nq:
            raise ValueError(f"Configuration vector dimension error: expected {self.model.nq}, got {q.shape[0]}")

        # Compute gravity term
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q)

        # Extract torques for arm joints
        arm_torques = tau_g[self.arm_joint_indices]

        return tau_g, arm_torques


def test_basic_configuration():
    """Test gravity compensation at basic configuration"""
    print("\n" + "="*60)
    print("Test 1: Basic Configuration (all joints at zero)")
    print("="*60)

    # URDF path (adjust according to actual location)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../model/pi_plus_arm6_head5_29/urdf/pi_plus_arm6_head5_29.urdf")

    # Create compensator
    compensator = StandaloneGravityCompensator(urdf_path)

    # Test configuration: all joints at zero
    q_zero = np.zeros(compensator.model.nq)

    tau_g_all, tau_g_arm = compensator.compute_gravity_torques(q_zero)

    print("\n[Results] Gravity compensation torques for all joints:")
    print(f"  Shape: {tau_g_all.shape}")
    print(f"  Max: {np.max(np.abs(tau_g_all)):.4f} Nm")
    print(f"  Norm: {np.linalg.norm(tau_g_all):.4f} Nm")

    print("\n[Results] Gravity compensation torques for arm joints:")
    for i, (name, torque) in enumerate(zip(compensator.ARM_JOINT_NAMES, tau_g_arm)):
        print(f"  {i+1:2d}. {name:25s}: {torque:8.4f} Nm")

    return compensator, tau_g_arm


def test_raised_arms():
    """Test configuration with raised arms"""
    print("\n" + "="*60)
    print("Test 2: Raised Arms Configuration")
    print("="*60)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../model/pi_plus_arm6_head5_29/urdf/pi_plus_arm6_head5_29.urdf")

    compensator = StandaloneGravityCompensator(urdf_path)

    # Create configuration: both arms raised forward 90 degrees
    q = np.zeros(compensator.model.nq)

    # Set shoulder pitch joints
    for joint_name, angle in [
        ('r_shoulder_pitch_joint', -np.pi/2),  # Right shoulder forward 90 deg
        ('r_shoulder_roll_joint', -np.pi/2),
        ('l_shoulder_pitch_joint', -np.pi/2),  # Left shoulder forward 90 deg
        ('l_shoulder_roll_joint', np.pi/2)
    ]:
        if joint_name in compensator.joint_name_to_id:
            joint_id = compensator.joint_name_to_id[joint_name]
            idx_q = compensator.model.joints[joint_id].idx_q
            q[idx_q] = angle

    tau_g_all, tau_g_arm = compensator.compute_gravity_torques(q)

    print("\n[Results] Gravity compensation torques for arm joints:")
    for i, (name, torque) in enumerate(zip(compensator.ARM_JOINT_NAMES, tau_g_arm)):
        print(f"  {i+1:2d}. {name:25s}: {torque:8.4f} Nm")

    return compensator, tau_g_arm


def test_multiple_configurations():
    """Test multiple configurations"""
    print("\n" + "="*60)
    print("Test 3: Multiple Configuration Comparison")
    print("="*60)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../model/pi_plus_arm6_head5_29/urdf/pi_plus_arm6_head5_29.urdf")

    compensator = StandaloneGravityCompensator(urdf_path)

    # Define multiple test configurations
    test_configs = {
        "Zero": np.zeros(compensator.model.nq),
        "R_Arm_30deg": np.zeros(compensator.model.nq),
        "R_Arm_60deg": np.zeros(compensator.model.nq),
        "R_Arm_90deg": np.zeros(compensator.model.nq),
    }

    # Set different angles
    for i, angle in enumerate([0, -np.pi/6, -np.pi/3, -np.pi/2]):
        config_name = list(test_configs.keys())[i]
        q = test_configs[config_name]
        if 'r_shoulder_pitch_joint' in compensator.joint_name_to_id:
            joint_id = compensator.joint_name_to_id['r_shoulder_pitch_joint']
            idx_q = compensator.model.joints[joint_id].idx_q
            q[idx_q] = angle

    # Compute and store results
    results = {}
    for config_name, q in test_configs.items():
        _, tau_g_arm = compensator.compute_gravity_torques(q)
        results[config_name] = tau_g_arm

    # Print comparison results
    print("\n[Results] Right shoulder pitch joint gravity torques at different configurations:")
    print(f"{'Configuration':<15s} {'r_shoulder_pitch (Nm)':>20s}")
    print("-" * 40)
    for config_name, tau_g_arm in results.items():
        print(f"{config_name:<15s} {tau_g_arm[0]:>20.4f}")

    return compensator, results


def plot_results(results):
    """Plot results"""
    if not HAS_MATPLOTLIB:
        print("\n[Info] matplotlib not installed, skipping plotting")
        return

    print("\n" + "="*60)
    print("Plotting Results")
    print("="*60)

    # Extract data
    config_names = list(results.keys())
    num_configs = len(config_names)
    num_joints = 12

    # Create data matrix
    torque_matrix = np.zeros((num_configs, num_joints))
    for i, config_name in enumerate(config_names):
        torque_matrix[i, :] = results[config_name]

    # Create figure
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Plot 1: Heatmap showing all configuration torques
    ax1 = axes[0]
    im = ax1.imshow(torque_matrix.T, aspect='auto', cmap='RdBu_r',
                     vmin=-np.max(np.abs(torque_matrix)),
                     vmax=np.max(np.abs(torque_matrix)))
    ax1.set_xlabel('Configuration')
    ax1.set_ylabel('Joint')
    ax1.set_title('Dual-Arm Gravity Compensation Torques Heatmap')
    ax1.set_xticks(range(num_configs))
    ax1.set_xticklabels(config_names, rotation=45, ha='right')
    ax1.set_yticks(range(num_joints))

    joint_labels = [
        'R_Sh_P', 'R_Sh_R', 'R_UA', 'R_Elb', 'R_Wr', 'R_Grip',
        'L_Sh_P', 'L_Sh_R', 'L_UA', 'L_Elb', 'L_Wr', 'L_Grip'
    ]
    ax1.set_yticklabels(joint_labels)
    plt.colorbar(im, ax=ax1, label='Torque (Nm)')

    # Plot 2: Right shoulder pitch joint variation
    ax2 = axes[1]
    shoulder_torques = torque_matrix[:, 0]  # Right shoulder pitch
    ax2.plot(range(num_configs), shoulder_torques, 'o-', linewidth=2, markersize=8)
    ax2.set_xlabel('Configuration')
    ax2.set_ylabel('Torque (Nm)')
    ax2.set_title('Right Shoulder Pitch Joint Gravity Compensation Torque Variation')
    ax2.set_xticks(range(num_configs))
    ax2.set_xticklabels(config_names, rotation=45, ha='right')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    plt.tight_layout()
    plt.savefig('/tmp/gravity_compensation_test.png', dpi=150, bbox_inches='tight')
    print(f"\n[Info] Plot saved to: /tmp/gravity_compensation_test.png")
    plt.show()


def main():
    """Main function"""
    print("\n" + "="*60)
    print("Dual-Arm Gravity Compensation Test Program (Standalone)")
    print("="*60)

    try:
        # Test 1: Basic configuration
        compensator1, tau1 = test_basic_configuration()

        # Test 2: Raised arms
        compensator2, tau2 = test_raised_arms()

        # Test 3: Multiple configuration comparison
        compensator3, results = test_multiple_configurations()

        # Plot results (if matplotlib available)
        plot_results(results)

        print("\n" + "="*60)
        print("All tests completed!")
        print("="*60)

        # Provide interactive testing option
        print("\n[Info] You can manually set joint angles for testing:")
        print("Example code:")
        print("  q = np.zeros(compensator3.model.nq)")
        print("  q[some_index] = np.pi/4  # Set some joint angle")
        print("  _, tau = compensator3.compute_gravity_torques(q)")
        print("  print(tau)")

        return compensator3

    except Exception as e:
        print(f"\n[Error] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    compensator = main()

    # Continue interactive testing here
    print("\n[Info] compensator object created, ready to use")
    print("      Example: compensator.compute_gravity_torques(q)")
