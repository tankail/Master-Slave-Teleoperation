#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>

#ifndef RELEASE
#include "robot.h"
#else
#include "livelybot_serial/hardware/robot.h"
#endif

/**
 * @brief Dual Arms Motor Controller with Bevel Gear Kinematics
 * 
 * This node controls both left and right arms simultaneously:
 * 1. Reads motor states from both arms (12 motors total)
 * 2. Publishes unified joint states for both arms
 * 3. Subscribes to gravity compensation torques for both arms
 * 4. Applies gravity compensation to both arms
 */
class DualArmsMotorController
{
public:
    DualArmsMotorController()
        : nh_("~")
        , publish_rate_(100)
        , control_rate_(400)
        , enable_gravity_comp_(true)
        , torque_scale_(1.0)
    {
        // Get parameters
        nh_.param("publish_rate", publish_rate_, 100);
        nh_.param("control_rate", control_rate_, 400);
        nh_.param("enable_gravity_comp", enable_gravity_comp_, true);
        nh_.param("torque_scale", torque_scale_, 1.0);
        nh_.param("debug", debug_, false);

        // Initialize robot object
        robot_.reset(new livelybot_serial::robot());

        // Initialize publisher
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
            "/dual_arms_joint_states", 10);

        // Subscribe to gravity compensation torques
        gravity_comp_sub_ = nh_.subscribe(
            "/dual_arms_gravity_compensation_torques",
            10,
            &DualArmsMotorController::gravityCompCallback,
            this);

        // Get motor count (should be 12: 6 for left arm + 6 for right arm)
        motor_count_ = robot_->Motors.size();
        ROS_INFO("Detected %ld motors", motor_count_);

        // Initialize joint names for both arms
        joint_names_ = {
            "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_shoulder_yaw_joint",
            "l_elbow_pitch_joint", "l_elbow_yaw_joint", "l_wrist_joint",
            "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_shoulder_yaw_joint",
            "r_elbow_pitch_joint", "r_elbow_yaw_joint", "r_wrist_joint"
        };

        // Initialize gravity compensation torques vector
        gravity_torques_.resize(joint_names_.size(), 0.0);

        // Send get motor state command
        robot_->send_get_motor_state_cmd();

        ROS_INFO("\033[1;32m=== Dual Arms Motor Controller Started ===\033[0m");
        ROS_INFO("Publish rate: %d Hz", publish_rate_);
        ROS_INFO("Control rate: %d Hz", control_rate_);
        ROS_INFO("Gravity compensation: %s", enable_gravity_comp_ ? "ENABLED" : "DISABLED");
        ROS_INFO("Torque scale: %.2f", torque_scale_);
    }

    void gravityCompCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Update gravity compensation torques for both arms
        for (size_t i = 0; i < joint_names_.size() && i < msg->effort.size(); ++i)
        {
            gravity_torques_[i] = msg->effort[i] * torque_scale_;
        }

        has_gravity_torque_ = true;

        if (debug_ && (++gravity_msg_count_ % 1000 == 0))
        {
            ROS_INFO("Gravity torques (L0-2, R0-2): [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f]",
                     gravity_torques_[0], gravity_torques_[1], gravity_torques_[2],
                     gravity_torques_[6], gravity_torques_[7], gravity_torques_[8]);
        }
    }

    /**
     * @brief Forward kinematics: motor positions -> joint positions
     * Handles bevel gear mechanism for both arms
     */
    void motorToJoint(const std::vector<double>& motor_pos, std::vector<double>& joint_pos)
    {
        joint_pos.resize(joint_names_.size(), 0.0);

        if (motor_pos.size() < 12) return;

        // Left arm (motors 0-5)
        joint_pos[0] = motor_pos[0];  // l_shoulder_pitch
        joint_pos[1] = (-motor_pos[1] + motor_pos[2]) / 2.0;  // l_shoulder_roll
        joint_pos[2] = (motor_pos[1] + motor_pos[2]) / 2.0;   // l_shoulder_yaw
        joint_pos[3] = (motor_pos[3] - motor_pos[4]) / 2.0;   // l_elbow_pitch
        joint_pos[4] = (-motor_pos[3] - motor_pos[4]) / 2.0;  // l_elbow_yaw
        joint_pos[5] = -motor_pos[5];  // l_wrist

        // Right arm (motors 6-11)
        joint_pos[6] = -motor_pos[6];  // r_shoulder_pitch
        joint_pos[7] = (motor_pos[7] - motor_pos[8]) / 2.0;   // r_shoulder_roll
        joint_pos[8] = (-motor_pos[7] - motor_pos[8]) / 2.0;  // r_shoulder_yaw
        joint_pos[9] = (motor_pos[9] - motor_pos[10]) / 2.0;  // r_elbow_pitch
        joint_pos[10] = (motor_pos[9] + motor_pos[10]) / 2.0; // r_elbow_yaw
        joint_pos[11] = -motor_pos[11];  // r_wrist
    }

    /**
     * @brief Inverse kinematics: joint torques -> motor torques
     */
    void jointToMotorTorque(const std::vector<double>& joint_torques, std::vector<double>& motor_torques)
    {
        motor_torques.resize(motor_count_, 0.0);

        if (joint_torques.size() < joint_names_.size()) return;

        // Left arm
        motor_torques[0] = joint_torques[0];
        motor_torques[1] = (-joint_torques[1] + joint_torques[2]) / 2.0;
        motor_torques[2] = (joint_torques[1] + joint_torques[2]) / 2.0;
        motor_torques[3] = (joint_torques[3] - joint_torques[4]) / 2.0;
        motor_torques[4] = (-joint_torques[3] - joint_torques[4]) / 2.0;
        motor_torques[5] = -joint_torques[5];

        // Right arm
        motor_torques[6] = -joint_torques[6];
        motor_torques[7] = (joint_torques[7] - joint_torques[8]) / 2.0;
        motor_torques[8] = (-joint_torques[7] - joint_torques[8]) / 2.0;
        motor_torques[9] = (joint_torques[9] + joint_torques[10]) / 2.0;
        motor_torques[10] = (-joint_torques[9] + joint_torques[10]) / 2.0;
        motor_torques[11] = -joint_torques[11];
    }

    void controlLoop()
    {
        ros::Rate rate(control_rate_);

        ROS_INFO("Waiting for gravity compensation torques...");
        while (ros::ok() && !has_gravity_torque_ && enable_gravity_comp_)
        {
            ros::Duration(0.1).sleep();
            ROS_INFO_THROTTLE(2.0, "Still waiting for gravity compensation...");
        }

        ROS_INFO("\033[1;32mControl loop started!\033[0m");

        while (ros::ok())
        {
            robot_->detect_motor_limit();

            {
                std::lock_guard<std::mutex> lock(data_mutex_);

                std::vector<double> motor_torques;
                jointToMotorTorque(gravity_torques_, motor_torques);

                for (size_t i = 0; i < robot_->Motors.size(); ++i)
                {
                    if (i < motor_torques.size())
                    {
                        double output_torque = enable_gravity_comp_ ? motor_torques[i] : 0.0;
                        auto current_state = robot_->Motors[i]->get_current_motor_state();

                        robot_->Motors[i]->pos_vel_tqe_kp_kd2(
                            current_state->position,
                            0.0,
                            output_torque,
                            0.0,
                            0.0
                        );
                    }
                }
            }

            robot_->motor_send_2();
            control_loop_count_++;
            rate.sleep();
        }
    }

    void publishLoop()
    {
        ros::Rate loop_rate(publish_rate_);

        while (ros::ok())
        {
            robot_->detect_motor_limit();

            sensor_msgs::JointState joint_state_msg;
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.name = joint_names_;
            joint_state_msg.position.resize(joint_names_.size());
            joint_state_msg.velocity.resize(joint_names_.size());
            joint_state_msg.effort.resize(joint_names_.size());

            {
                std::lock_guard<std::mutex> lock(data_mutex_);

                std::vector<double> motor_positions(motor_count_);
                std::vector<double> motor_velocities(motor_count_);
                std::vector<double> motor_efforts(motor_count_);

                for (size_t i = 0; i < motor_count_; ++i)
                {
                    if (i < robot_->Motors.size())
                    {
                        auto motor_state = robot_->Motors[i]->get_current_motor_state();
                        motor_positions[i] = (motor_state->position != 999.0) ? motor_state->position : 0.0;
                        motor_positions[i] = std::max(-3.14, std::min(motor_positions[i], 3.14));
                        motor_velocities[i] = motor_state->velocity;
                        motor_efforts[i] = motor_state->torque;
                    }
                }

                std::vector<double> joint_positions;
                motorToJoint(motor_positions, joint_positions);

                for (size_t i = 0; i < joint_names_.size(); ++i)
                {
                    joint_state_msg.position[i] = joint_positions[i];
                    if (i < motor_count_)
                    {
                        joint_state_msg.velocity[i] = motor_velocities[i];
                        joint_state_msg.effort[i] = motor_efforts[i];
                    }
                }
            }

            joint_state_pub_.publish(joint_state_msg);
            publish_count_++;

            if (publish_count_ % 1000 == 0)
            {
                ROS_INFO("Published %ld messages, Control loops: %ld",
                         publish_count_, control_loop_count_);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void setDebug(bool debug) { debug_ = debug; }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber gravity_comp_sub_;
    std::unique_ptr<livelybot_serial::robot> robot_;
    std::mutex data_mutex_;

    int publish_rate_;
    int control_rate_;
    bool enable_gravity_comp_;
    double torque_scale_;
    bool debug_ = false;

    size_t motor_count_;
    std::vector<std::string> joint_names_;
    std::vector<double> gravity_torques_;

    bool has_gravity_torque_ = false;
    long publish_count_ = 0;
    long control_loop_count_ = 0;
    long gravity_msg_count_ = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arms_motor_controller");

    try
    {
        DualArmsMotorController controller;

        bool debug = false;
        ros::NodeHandle nh("~");
        nh.param("debug", debug, false);
        controller.setDebug(debug);

        std::thread control_thread(&DualArmsMotorController::controlLoop, &controller);
        controller.publishLoop();
        control_thread.join();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
