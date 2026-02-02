#ifndef GRAVITY_COMPENSATION_CONTROLLER_H
#define GRAVITY_COMPENSATION_CONTROLLER_H


#include <pinocchio/fwd.hpp>  // 必须最先包含
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>  // 重力补偿使用 rnea 算法
#include <pinocchio/algorithm/crba.hpp>  // 或者 crba 算法
#include <pinocchio/algorithm/jacobian.hpp>
// Pinocchio includes
// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/algorithm/gravity.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>



class GravityCompensationController
{
public:
    GravityCompensationController();
    ~GravityCompensationController();
    
    bool initialize();
    void run();
    void shutdown();

private:
    // Joint names
    static const std::vector<std::string> ARM_JOINT_NAMES;
    
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscribers and publishers
    ros::Subscriber joint_state_sub_;
    ros::Publisher gravity_comp_pub_;
    
    // Parameters
    int publish_rate_;
    bool enable_compensation_;
    double compensation_scale_;
    std::string urdf_path_;
    
    // Pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;
    
    // Joint mapping
    std::map<std::string, int> joint_name_to_id_;
    std::vector<int> arm_joint_indices_;
    
    // State
    sensor_msgs::JointState::ConstPtr current_joint_state_;
    std::mutex state_mutex_;
    
    // Statistics
    unsigned long msg_count_;
    unsigned long error_count_;
    
    // Callbacks
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
    // Helper methods
    std::string getDefaultURDFPath();
    void buildJointMapping();
    std::vector<double> computeGravityCompensation(const std::vector<double>& joint_positions);
    void publishStatistics();
};

#endif // GRAVITY_COMPENSATION_CONTROLLER_H