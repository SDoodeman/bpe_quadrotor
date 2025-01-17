#pragma once

#include <map>
#include <array>
#include <autopilot/mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <bpe_msgs/msg/bpe_statistics.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "pegasus_msgs/msg/control_attitude.hpp"
#include "pegasus_msgs/msg/control_position.hpp"
#include <std_msgs/msg/float64.hpp>

namespace autopilot {

class BpeMode2 : public autopilot::Mode {

public:

    ~BpeMode2();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

    // Get the current vehicle state
    void update_vehicle_state();

    // Auxiliar method to update the desired trajectory
    void initialize_trajectory();
    void trajectory_generation(double dt);

    // Get the state of the other agents
    void target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int drone_id);

protected:

    // General drone parameters
    int drone_id_;
    double mass_;

    // Only used by the leader drone
    double total_time_{0};

    // Tracking Gains
    double Kp_{0.0};
    double Kv_{0.0};
    double Kr_{0.0};

    // Colision avoidance Gains
    double Ko_{0.0};
    double r_{0.0};

    // Trajectory parameters
    double z_min_{0.0};
    double z_max_{0.0};
    double A_offset_{0.0};
    double frequency_{0.0};
    double A_min_{0.0};
    double leader_z_min_{0.0};
    double leader_z_max_{0.0};

    // Adjacency matrix
    const static size_t n_agents_{3};
    Eigen::Matrix3i aij_{Eigen::Matrix3i::Zero()};
    int N_following_{0};
    
    // Defines the id of the leader vehicle (by default the first drone is the leader and only tracks the trajectory)
    std::array<int, 3> drone_ids_{0,0,0};
    std::map<int, int> graph_ids_;

    // Subscribers vector for the position of each drone
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> target_subs_;

    // --------------------
    // Trajectory variables
    // --------------------
    std::array<Eigen::Vector3d, n_agents_> pdes_;
    std::array<Eigen::Vector3d, n_agents_> vdes_;
    std::array<Eigen::Vector3d, n_agents_> udes_;
    std::array<Eigen::Vector3d, n_agents_> jdes_;

    // -----------------
    // State of each agent in the network
    // ----------------
    std::array<Eigen::Vector3d, n_agents_> P_;
    std::array<Eigen::Vector3d, n_agents_> V_;

    // The attitude of this vehicle
    Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};  // Attitude of this vehicle

    // Statistics message for the BPE Mode and publisher
    bpe_msgs::msg::BpeStatistics statistics_msg_;
    rclcpp::Publisher<bpe_msgs::msg::BpeStatistics>::SharedPtr statistics_publisher_;

    // ROS2 messages
    pegasus_msgs::msg::ControlAttitude desired_attitude_msg_;
    pegasus_msgs::msg::ControlAttitude desired_attitude_rate_msg_;
    pegasus_msgs::msg::ControlPosition desired_position_msg_;
    std_msgs::msg::Float64 position_error_msg_;
    std_msgs::msg::Float64 total_time_msg_;
    pegasus_msgs::msg::ControlPosition collision_avoidance_force_msg_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr desired_attitude_publisher_;
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr desired_attitude_rate_publisher_;
    rclcpp::Publisher<pegasus_msgs::msg::ControlPosition>::SharedPtr desired_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_time_publisher_;
    rclcpp::Publisher<pegasus_msgs::msg::ControlPosition>::SharedPtr collision_avoidance_force_publisher_;
};
}