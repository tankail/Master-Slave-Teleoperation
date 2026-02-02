#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <cmath>
#include <mutex>
#include <vector>
#include <string>
#include <map>

/**
 * @brief Dual Arms Gravity Compensation Calculator
 * 
 * This node:
 * 1. Subscribes to dual arms joint states from /dual_arms_joint_states
 * 2. Uses Pinocchio to compute gravity compensation torques for both arms
 * 3. Publishes compensation torques to /dual_arms_gravity_compensation_torques
 */
class DualArmsGravityCompensation
{
public:
    DualArmsGravityCompensation()
        : nh_()
        , private_nh_("~")
        , msg_count_(0)
        , error_count_(0)
    {
        ROS_INFO("\033[1;32mInitializing Dual Arms Gravity Compensation Calculator\033[0m");
    }

    ~DualArmsGravityCompensation()
    {
        shutdown();
    }

    bool initialize()
    {
        try {
            // Get parameters
            private_nh_.param("publish_rate", publish_rate_, 100);
            private_nh_.param("enable_compensation", enable_compensation_, true);
            private_nh_.param("compensation_scale", compensation_scale_, 1.0);

            // Get URDF path
            private_nh_.param<std::string>("urdf_path", urdf_path_, 
                "/home/hightorque/urdf_models/pi_plus_36dof_260107.urdf");

            ROS_INFO("Loading URDF from: %s", urdf_path_.c_str());

            // Check if URDF file exists
            std::ifstream urdf_file(urdf_path_);
            if (!urdf_file.good()) {
                ROS_ERROR("URDF file not found: %s", urdf_path_.c_str());
                return false;
            }
            urdf_file.close();

            // Load URDF and create model
            pinocchio::urdf::buildModel(urdf_path_, model_);
            data_ = pinocchio::Data(model_);

            ROS_INFO("Pinocchio model loaded: %d DOFs, %d joints",
                     model_.nq, model_.njoints);

            // Define both arms joint names
            dual_arms_joint_names_ = {
                "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_shoulder_yaw_joint",
                "l_elbow_pitch_joint", "l_elbow_yaw_joint", "l_wrist_joint",
                "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_shoulder_yaw_joint",
                "r_elbow_pitch_joint", "r_elbow_yaw_joint", "r_wrist_joint"
            };

            // Build joint mapping
            buildJointMapping();

            // Initialize publishers and subscribers
            joint_state_sub_ = nh_.subscribe("/dual_arms_joint_states",
                                            10,
                                            &DualArmsGravityCompensation::jointStateCallback,
                                            this);

            gravity_comp_pub_ = nh_.advertise<sensor_msgs::JointState>(
                "/dual_arms_gravity_compensation_torques", 10);

            ROS_INFO("Dual Arms Gravity Compensation Calculator Initialized");
            ROS_INFO("Publish rate: %d Hz", publish_rate_);
            ROS_INFO("Compensation enabled: %s", enable_compensation_ ? "true" : "false");
            ROS_INFO("Compensation scale: %.2f", compensation_scale_);

            return true;

        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize calculator: %s", e.what());
            return false;
        }
    }

    void buildJointMapping()
    {
        joint_name_to_id_.clear();
        dual_arms_joint_indices_.clear();

        // Create name to ID mapping
        for (size_t joint_id = 1; joint_id < (size_t)model_.names.size(); ++joint_id) {
            const std::string& joint_name = model_.names[joint_id];
            joint_name_to_id_[joint_name] = joint_id;
        }

        // Find configuration indices for both arms joints
        for (const auto& joint_name : dual_arms_joint_names_) {
            auto it = joint_name_to_id_.find(joint_name);
            if (it != joint_name_to_id_.end()) {
                int joint_id = it->second;
                int idx_q = model_.joints[joint_id].idx_q();
                ROS_INFO("Mapped %s to index %d", joint_name.c_str(), idx_q);
                dual_arms_joint_indices_.push_back(idx_q);
            } else {
                ROS_WARN("Joint not found in URDF: %s", joint_name.c_str());
                dual_arms_joint_indices_.push_back(-1);
            }
        }

        int valid_joints = 0;
        for (int idx : dual_arms_joint_indices_) {
            if (idx >= 0) valid_joints++;
        }

        ROS_INFO("Mapped %d dual arms joints", valid_joints);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_joint_state_ = msg;
    }

    std::vector<double> computeGravityCompensation(const std::vector<double>& joint_positions)
    {
        // Create full configuration vector
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);

        // Fill in both arms joint positions
        for (size_t i = 0; i < dual_arms_joint_indices_.size(); ++i) {
            int idx_q = dual_arms_joint_indices_[i];
            if (idx_q >= 0 && i < joint_positions.size()) {
                q[idx_q] = joint_positions[i];
            }
        }

        // Compute gravity torques using Pinocchio
        Eigen::VectorXd tau_g = pinocchio::computeGeneralizedGravity(model_, data_, q);

        // Extract torques for both arms joints
        std::vector<double> arms_torques(dual_arms_joint_names_.size(), 0.0);
        for (size_t i = 0; i < dual_arms_joint_indices_.size(); ++i) {
            int idx_q = dual_arms_joint_indices_[i];
            if (idx_q >= 0 && idx_q < tau_g.size()) {
                arms_torques[i] = tau_g[idx_q];
            }
        }

        // Apply scale factor
        for (double& torque : arms_torques) {
            torque *= compensation_scale_;
        }

        return arms_torques;
    }

    void run()
    {
        if (!initialize()) {
            ROS_ERROR("Failed to initialize calculator");
            return;
        }

        ros::Rate rate(publish_rate_);

        ROS_INFO("Waiting for first joint state message...");

        while (ros::ok() && !current_joint_state_) {
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("Received first joint state, starting gravity compensation calculation");

        while (ros::ok()) {
            try {
                sensor_msgs::JointState::ConstPtr joint_state;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    joint_state = current_joint_state_;
                }

                if (!joint_state) {
                    rate.sleep();
                    ros::spinOnce();
                    continue;
                }

                std::vector<double> gravity_torques(dual_arms_joint_names_.size(), 0.0);

                if (enable_compensation_ &&
                    joint_state->position.size() >= dual_arms_joint_names_.size()) {

                    std::vector<double> arms_positions(
                        joint_state->position.begin(),
                        joint_state->position.begin() + dual_arms_joint_names_.size()
                    );

                    gravity_torques = computeGravityCompensation(arms_positions);
                }

                sensor_msgs::JointState output_msg;
                output_msg.header.stamp = ros::Time::now();
                output_msg.name = dual_arms_joint_names_;

                if (joint_state->position.size() >= dual_arms_joint_names_.size()) {
                    output_msg.position.assign(
                        joint_state->position.begin(),
                        joint_state->position.begin() + dual_arms_joint_names_.size()
                    );
                } else {
                    output_msg.position.assign(dual_arms_joint_names_.size(), 0.0);
                }

                if (joint_state->velocity.size() >= dual_arms_joint_names_.size()) {
                    output_msg.velocity.assign(
                        joint_state->velocity.begin(),
                        joint_state->velocity.begin() + dual_arms_joint_names_.size()
                    );
                } else {
                    output_msg.velocity.assign(dual_arms_joint_names_.size(), 0.0);
                }

                output_msg.effort = gravity_torques;

                gravity_comp_pub_.publish(output_msg);

                msg_count_++;
                if (msg_count_ % 1000 == 0) {
                    publishStatistics();

                    if (!gravity_torques.empty()) {
                        ROS_INFO("Sample gravity torques L[0-2]: [%.3f, %.3f, %.3f] R[0-2]: [%.3f, %.3f, %.3f]",
                                gravity_torques[0], gravity_torques[1], gravity_torques[2],
                                gravity_torques[6], gravity_torques[7], gravity_torques[8]);
                    }
                }

            } catch (const std::exception& e) {
                error_count_++;
                ROS_ERROR_THROTTLE(1.0, "Error in calculation loop: %s", e.what());
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    void publishStatistics()
    {
        ROS_INFO("Published %lu messages, %lu errors", msg_count_, error_count_);
    }

    void shutdown()
    {
        ROS_INFO("Shutting down dual arms gravity compensation calculator");
        ROS_INFO("Total messages: %lu, Errors: %lu", msg_count_, error_count_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher gravity_comp_pub_;

    int publish_rate_;
    bool enable_compensation_;
    double compensation_scale_;
    std::string urdf_path_;

    pinocchio::Model model_;
    pinocchio::Data data_;

    std::vector<std::string> dual_arms_joint_names_;
    std::map<std::string, int> joint_name_to_id_;
    std::vector<int> dual_arms_joint_indices_;

    sensor_msgs::JointState::ConstPtr current_joint_state_;
    std::mutex state_mutex_;

    unsigned long msg_count_;
    unsigned long error_count_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arms_gravity_compensation");

    try
    {
        DualArmsGravityCompensation calculator;
        calculator.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
