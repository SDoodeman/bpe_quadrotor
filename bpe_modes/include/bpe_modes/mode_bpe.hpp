#pragma once

#include <autopilot/mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "pegasus_msgs/msg/control_attitude.hpp"
#include "pegasus_msgs/msg/control_position.hpp"

namespace autopilot {

class BpeMode : public autopilot::Mode {

public:

    enum OperationMode {BPE_OFF, BPE_ON};

    ~BpeMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

    void update_vehicle_state();
    void target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int id);

protected:

    // Gains
    double Kp{1.0};
    double Kv{1.0};
    double Kr{1.0};

    // General parameters
    double g{9.81};
    double mass;
    double thrust{0};
    double t{0};
    int n_agents{3};
    int drone_id{1}; // TODO ???
    int aij[3][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };

    // Subscribers vector for the position of each drone
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> target_subs_;

    // Trajectory variables
    Eigen::Vector3d P{Eigen::Vector3d::Zero()};     // Position
    Eigen::Vector3d V{Eigen::Vector3d::Zero()};     // Velocity
    Eigen::Matrix3d R{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    std::vector<Eigen::Vector3d> pdes{static_cast<size_t>(n_agents), Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> vdes{static_cast<size_t>(n_agents), Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> udes{static_cast<size_t>(n_agents), Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> jdes{static_cast<size_t>(n_agents), Eigen::Vector3d::Zero()};

    // Control variables
    Eigen::Vector3d u{Eigen::Vector3d::Zero()};
    Eigen::Vector3d TRde3{Eigen::Vector3d::Zero()};
    Eigen::Vector3d Rde3{Eigen::Vector3d::Zero()};
    Eigen::Vector3d e3{0.0, 0.0, 1.0};
    Eigen::Vector3d pij{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gij{Eigen::Vector3d::Zero()};
    Eigen::Vector3d pijd{Eigen::Vector3d::Zero()};
    Eigen::Vector3d attitude = Eigen::Vector3d::Zero();
    Eigen::Vector3d attitude_rate = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> P_other{static_cast<size_t>(n_agents), Eigen::Vector3d::Zero()};

    // Control inputs to apply to the vehicle
    Eigen::Vector3d velocity_;
    Eigen::Vector3d acel_;

    // Operation Mode
    OperationMode operation_mode_{BPE_OFF};

    // ROS2 messages
    pegasus_msgs::msg::ControlAttitude desired_attitude_msg_;
    pegasus_msgs::msg::ControlAttitude desired_attitude_rate_msg_;
    pegasus_msgs::msg::ControlPosition desired_position_msg_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr desired_attitude_publisher_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr desired_attitude_rate_publisher_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::ControlPosition>::SharedPtr desired_position_publisher_{nullptr};
};
}