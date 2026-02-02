#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#ifndef RELEASE
#include "robot.h"
#else
#include "livelybot_serial/hardware/robot.h"
#endif

#include <iostream>
#include <vector>
#include <mutex>
#include <algorithm>

/**
 * @brief Impedance Controller for Dual Arms Slave
 * 
 * This controller implements impedance control for both arms simultaneously
 */
class DualArmsImpedanceController
{
public:
    DualArmsImpedanceController() 
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // Get impedance control parameters
        private_nh.param("stiffness_kp", stiffness_kp_, 10.0);
        private_nh.param("damping_kd", damping_kd_, 1.0);
        private_nh.param("max_torque", max_torque_, 10.0);
        private_nh.param("control_rate", control_rate_, 400);
        private_nh.param("enable_feedforward", enable_feedforward_, false);
        
        robot_.reset(new livelybot_serial::robot());
        
        // Subscribe to master joint states (unified topic)
        joint_state_sub_ = nh.subscribe(
            "/dual_arms_joint_states", 
            10, 
            &DualArmsImpedanceController::jointStateCallback, 
            this);
        
        // Publisher for slave joint states (for monitoring/visualization)
        slave_state_pub_ = nh.advertise<sensor_msgs::JointState>(
            "/slave_dual_arms_joint_states", 10);
        
        motor_count_ = robot_->Motors.size();  // Should be 12
        target_positions_.resize(motor_count_, 0.0);
        target_velocities_.resize(motor_count_, 0.0);
        current_positions_.resize(motor_count_, 0.0);
        current_velocities_.resize(motor_count_, 0.0);
        
        ROS_INFO("\033[1;32m=== Dual Arms Impedance Controller Started ===\033[0m");
        ROS_INFO("Motors: %ld", motor_count_);
        ROS_INFO("Stiffness (Kp): %.2f", stiffness_kp_);
        ROS_INFO("Damping (Kd): %.2f", damping_kd_);
        ROS_INFO("Max Torque: %.2f Nm", max_torque_);
        ROS_INFO("Control Rate: %d Hz", control_rate_);
        ROS_INFO("\033[1;33mPosition Limits Enabled for All Motors\033[0m");
    }
    
    /**
     * @brief Convert joint space to motor space for both arms
     */
    void jointToMotor(const sensor_msgs::JointState::ConstPtr& msg, 
                      std::vector<double>& motor_pos,
                      std::vector<double>& motor_vel)
    {
        motor_pos.resize(12, 0.0);
        motor_vel.resize(12, 0.0);

        if (msg->position.size() < 12) return;

        // Left arm (motors 0-5)
        motor_pos[0] = msg->position[0];  // l_shoulder_pitch
        motor_vel[0] = msg->velocity.size() > 0 ? msg->velocity[0] : 0.0;

        motor_pos[1] = msg->position[2] - msg->position[1];  // motor2
        motor_pos[2] = msg->position[2] + msg->position[1];  // motor3
        
        if (msg->velocity.size() >= 3) {
            motor_vel[1] = msg->velocity[2] - msg->velocity[1];
            motor_vel[2] = msg->velocity[2] + msg->velocity[1];
        }

        motor_pos[3] = msg->position[3] - msg->position[4];  // motor4
        motor_pos[4] = -msg->position[3] - msg->position[4]; // motor5
        
        if (msg->velocity.size() >= 5) {
            motor_vel[3] = msg->velocity[3] - msg->velocity[4];
            motor_vel[4] = -msg->velocity[3] - msg->velocity[4];
        }

        motor_pos[5] = -msg->position[5];  // l_wrist
        motor_vel[5] = msg->velocity.size() > 5 ? -msg->velocity[5] : 0.0;

        // Right arm (motors 6-11)
        motor_pos[6] = -msg->position[6];  // r_shoulder_pitch
        motor_vel[6] = msg->velocity.size() > 6 ? -msg->velocity[6] : 0.0;

        motor_pos[7] = msg->position[7] - msg->position[8];   // motor8 (FIXED)
        motor_pos[8] = -msg->position[7] - msg->position[8];  // motor9 (FIXED)
        
        if (msg->velocity.size() >= 9) {
            motor_vel[7] = msg->velocity[7] - msg->velocity[8];
            motor_vel[8] = -msg->velocity[7] - msg->velocity[8];
        }

        motor_pos[9] = msg->position[9] + msg->position[10];   // motor10
        motor_pos[10] = -msg->position[9] + msg->position[10]; // motor11
        
        if (msg->velocity.size() >= 11) {
            motor_vel[9] = msg->velocity[9] + msg->velocity[10];
            motor_vel[10] = -msg->velocity[9] + msg->velocity[10];
        }

        motor_pos[11] = -msg->position[11];  // r_wrist
        motor_vel[11] = msg->velocity.size() > 11 ? -msg->velocity[11] : 0.0;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        std::vector<double> motor_pos, motor_vel;
        jointToMotor(msg, motor_pos, motor_vel);

        // Update target positions and velocities
        for (size_t i = 0; i < motor_count_ && i < motor_pos.size(); ++i)
        {
            target_positions_[i] = motor_pos[i];
            target_velocities_[i] = motor_vel[i];
        }
        
        new_command_ = true;
    }
    
    /**
     * @brief Update current motor states directly from robot hardware
     */
    void updateCurrentStates()
    {
        for (size_t i = 0; i < robot_->Motors.size(); ++i)
        {
            auto state = robot_->Motors[i]->get_current_motor_state();
            current_positions_[i] = state->position;
            current_velocities_[i] = state->velocity;  // Velocity is read directly from motors
        }
    }
    
    /**
     * @brief Publish slave joint states for monitoring/visualization
     */
    void publishSlaveState()
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {
            "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_shoulder_yaw_joint",
            "l_elbow_pitch_joint", "l_elbow_yaw_joint", "l_wrist_joint",
            "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_shoulder_yaw_joint",
            "r_elbow_pitch_joint", "r_elbow_yaw_joint", "r_wrist_joint"
        };
        
        msg.position.resize(12);
        msg.velocity.resize(12);
        
        // Left arm - convert motor space back to joint space
        msg.position[0] = current_positions_[0];
        msg.velocity[0] = current_velocities_[0];
        
        msg.position[1] = (-current_positions_[1] + current_positions_[2]) / 2.0;
        msg.position[2] = (current_positions_[1] + current_positions_[2]) / 2.0;
        msg.velocity[1] = (-current_velocities_[1] + current_velocities_[2]) / 2.0;
        msg.velocity[2] = (current_velocities_[1] + current_velocities_[2]) / 2.0;
        
        msg.position[3] = (current_positions_[3] - current_positions_[4]) / 2.0;
        msg.position[4] = (-current_positions_[3] - current_positions_[4]) / 2.0;
        msg.velocity[3] = (current_velocities_[3] - current_velocities_[4]) / 2.0;
        msg.velocity[4] = (-current_velocities_[3] - current_velocities_[4]) / 2.0;
        
        msg.position[5] = -current_positions_[5];
        msg.velocity[5] = -current_velocities_[5];

        // Right arm - convert motor space back to joint space
        msg.position[6] = -current_positions_[6];
        msg.velocity[6] = -current_velocities_[6];
        
        msg.position[7] = (current_positions_[7] - current_positions_[8]) / 2.0;
        msg.position[8] = (-current_positions_[7] - current_positions_[8]) / 2.0;
        msg.velocity[7] = (current_velocities_[7] - current_velocities_[8]) / 2.0;
        msg.velocity[8] = (-current_velocities_[7] - current_velocities_[8]) / 2.0;
        
        msg.position[9] = (current_positions_[9] - current_positions_[10]) / 2.0;
        msg.position[10] = (current_positions_[9] + current_positions_[10]) / 2.0;
        msg.velocity[9] = (current_velocities_[9] - current_velocities_[10]) / 2.0;
        msg.velocity[10] = (current_velocities_[9] + current_velocities_[10]) / 2.0;
        
        msg.position[11] = -current_positions_[11];
        msg.velocity[11] = -current_velocities_[11];
        
        slave_state_pub_.publish(msg);
    }
    
    void run()
    {
        ros::Rate rate(control_rate_);
        ros::Rate publish_rate(100);
        
        int publish_counter = 0;
        int publish_divisor = control_rate_ / 100;
        
        ROS_INFO("\033[1;32mSTART DUAL ARMS IMPEDANCE CONTROL\033[0m");
        
        while (ros::ok())
        {
            ros::spinOnce();
            robot_->detect_motor_limit();
            
            // Update current states directly from motors (includes velocity)
            updateCurrentStates();
            
            if (new_command_)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                
                for (size_t i = 0; i < robot_->Motors.size(); ++i)
                {
                    if (i < target_positions_.size())
                    {
                        double limited_target_pos = target_positions_[i];
                        
                        // Apply position limits for all motors
                        // Motor 0: l_shoulder_pitch_joint
                        if (i == 0)
                        {
                            limited_target_pos = std::max(-2.8, std::min(2.8, target_positions_[i]));
                        }
                        // Motor 1: l_shoulder_roll/yaw combined (motor2)
                        else if (i == 1)
                        {
                            limited_target_pos = std::max(-1.5, std::min(1.5, target_positions_[i]));
                        }
                        // Motor 2: l_shoulder_roll/yaw combined (motor3)
                        else if (i == 2)
                        {
                            limited_target_pos = std::max(-3.14, std::min(3.14, target_positions_[i]));
                        }
                        // Motor 3: l_elbow combined (motor4)
                        else if (i == 3)
                        {
                            limited_target_pos = std::max(-2.1, std::min(2.1, target_positions_[i]));
                        }
                        // Motor 4: l_elbow combined (motor5)
                        else if (i == 4)
                        {
                            limited_target_pos = std::max(-3.0, std::min(3.0, target_positions_[i]));
                        }
                        // Motor 5: l_wrist_joint
                        else if (i == 5)
                        {
                            limited_target_pos = std::max(-0.5, std::min(0.5, target_positions_[i]));
                        }
                        // Motor 6: r_shoulder_pitch_joint
                        else if (i == 6)
                        {
                            limited_target_pos = std::max(-2.8, std::min(2.8, target_positions_[i]));
                        }
                        // Motor 7: r_shoulder_roll/yaw combined (motor8)
                        else if (i == 7)
                        {
                            limited_target_pos = std::max(-1.5, std::min(1.5, target_positions_[i]));
                        }
                        // Motor 8: r_shoulder_roll/yaw combined (motor9)
                        else if (i == 8)
                        {
                            limited_target_pos = std::max(-3.14, std::min(3.14, target_positions_[i]));
                        }
                        // Motor 9: r_elbow combined (motor10)
                        else if (i == 9)
                        {
                            limited_target_pos = std::max(-2.1, std::min(2.1, target_positions_[i]));
                        }
                        // Motor 10: r_elbow combined (motor11)
                        else if (i == 10)
                        {
                            limited_target_pos = std::max(-3.0, std::min(3.0, target_positions_[i]));
                        }
                        // Motor 11: r_wrist_joint
                        else if (i == 11)
                        {
                            limited_target_pos = std::max(-0.5, std::min(0.5, target_positions_[i]));
                        }
                        
                        double pos_error = limited_target_pos - current_positions_[i];
                        double vel_error = target_velocities_[i] - current_velocities_[i];
                        double torque = stiffness_kp_ * pos_error + damping_kd_ * vel_error;
                        
                        if (torque > max_torque_) torque = max_torque_;
                        if (torque < -max_torque_) torque = -max_torque_;
                        
                        robot_->Motors[i]->pos_vel_tqe_kp_kd2(
                            limited_target_pos,
                            target_velocities_[i],
                            torque,
                            stiffness_kp_,
                            damping_kd_
                        );
                    }
                }
            }
            
            robot_->motor_send_2();
            
            if (++publish_counter >= publish_divisor) {
                publishSlaveState();
                publish_counter = 0;
            }
            
            rate.sleep();
        }
        
        ROS_INFO_STREAM("END");
    }
    
private:
    ros::Subscriber joint_state_sub_;
    ros::Publisher slave_state_pub_;
    std::unique_ptr<livelybot_serial::robot> robot_;
    std::mutex data_mutex_;
    
    double stiffness_kp_;
    double damping_kd_;
    double max_torque_;
    int control_rate_;
    bool enable_feedforward_;
    
    size_t motor_count_;
    std::vector<double> target_positions_;
    std::vector<double> target_velocities_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    bool new_command_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arms_impedance_controller");
    
    DualArmsImpedanceController controller;
    controller.run();
    
    return 0;
}
