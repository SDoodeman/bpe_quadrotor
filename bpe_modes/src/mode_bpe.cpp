#include "pegasus_utils/rotations.hpp"
#include "bpe_modes/mode_bpe.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// TODO make sure to know which drone we are
// TODO make sure to get the positions of the other drones


namespace autopilot {

BpeMode::~BpeMode() {}

void BpeMode::initialize() {
    
    // Getting the drone ID and subscribing to the state of the other drones on the network
    drone_id = get_vehicle_constants().id;
    
    aij[1][0] = 1;
    aij[2][1] = 1;

    // for (size_t i = 0; i < n_agents; i++) {
    //     if (aij[drone_id-1][i]) {
    //         target_subs_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
    //             "/drone" + std::to_string(i+1) + "/fmu/filter/state", 
    //             rclcpp::SensorDataQoS(), 
    //             std::bind(&BpeMode::target_state_callback, this, std::placeholders::_1, i)
    //         ));
    //     }
    // }
    for (size_t i = 0; i < n_agents; i++) {
        if (aij[drone_id-1][i]) {
            target_subs_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
                "/drone" + std::to_string(i+1) + "/fmu/filter/state", 
                rclcpp::SensorDataQoS(), 
                [this, i](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    this->target_state_callback(msg, i);  // Capture i and pass to callback
                }
            ));
        }
    }

    // Load the gains of the controller (does not work??)
    node_->declare_parameter<double>("autopilot.BpeMode.gains.Kp", 2.0);
    node_->declare_parameter<double>("autopilot.BpeMode.gains.Kv", 0.5);
    node_->declare_parameter<double>("autopilot.BpeMode.gains.Kr", 10.0);

    mass = get_vehicle_constants().mass;

    for (size_t i = 0; i < n_agents; i++) {
        pdes[i] = Eigen::Vector3d(0.0, 0.0, -1.5 + 0.5 * drone_id);
    }

    // Initialize the ROS 2 subscribers to the control topics
    node_->declare_parameter<std::string>("autopilot.BpeController.publishers.control_attitude", "desired_control_attitude");
    node_->declare_parameter<std::string>("autopilot.BpeController.publishers.control_attitude_rate", "desired_control_attitude_rate");
    node_->declare_parameter<std::string>("autopilot.BpeController.publishers.control_position", "desired_control_position");

    // Create the publishers
    desired_attitude_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.BpeController.publishers.control_attitude").as_string(), rclcpp::SensorDataQoS());
    desired_attitude_rate_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.BpeController.publishers.control_attitude_rate").as_string(), rclcpp::SensorDataQoS());
    desired_position_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlPosition>(node_->get_parameter("autopilot.BpeController.publishers.control_position").as_string(), rclcpp::SensorDataQoS());

    Kp = node_->get_parameter("autopilot.BpeMode.gains.Kp").as_double();
    Kv = node_->get_parameter("autopilot.BpeMode.gains.Kv").as_double();
    Kr = node_->get_parameter("autopilot.BpeMode.gains.Kr").as_double();

    if (drone_id==1) {
        Kp = 5.0;
        Kv = 4.0; // Above 5/6 is unstable
        Kr = 10.0;
    } else {
        Kp = 2.0;
        Kv = 3.0;
        Kr = 8;
    }

    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kp: %f", Kp);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kv: %f", Kv);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kr: %f", Kr);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode initialized");
}

bool BpeMode::enter() {
    return true;
}

void BpeMode::update(double dt) {
    double A;
    double omega;
    
    // Get the current state of the vehicle
    update_vehicle_state();

    // Unknown variables!
    t += dt;

    for (size_t i = 0; i < n_agents; i++)
    {
        A = double(i+1);
        omega = 3.141592 * 2 / 10;
        pdes[i][0] = A*sin(omega*t - i*3.1415/2);
        pdes[i][1] = A*cos(omega*t - i*3.1415/2);
        vdes[i][0] = omega*A*cos(omega*t - i*3.1415/2);
        vdes[i][1] = -omega*A*sin(omega*t - i*3.1415/2);
        udes[i][0] = -omega*omega*A*sin(omega*t - i*3.1415/2);
        udes[i][1] = -omega*omega*A*cos(omega*t - i*3.1415/2);
        jdes[i][0] = -omega*omega*omega*A*cos(omega*t - i*3.1415/2);
        jdes[i][1] = omega*omega*omega*A*sin(omega*t - i*3.1415/2);
    }

    if (drone_id==1) {
        u = udes[0] - Kp*(P - pdes[0]) - Kv*(V - vdes[0]);
    } else {
        u = udes[drone_id-1] - Kv * (V - vdes[drone_id-1]);
        for (size_t j = 0; j < n_agents; j++)
        {
            if (aij[drone_id-1][j])
            {
                pij = P_other[j] - P;
                gij = pij.normalized();
                pijd = pdes[j] - pdes[drone_id-1];
                u = u - Kp*(pijd - gij * gij.dot(pijd));
            }   
        }  
    }

    TRde3 = mass*g*e3 - mass*u;

    thrust = TRde3.norm();
    Rde3 = TRde3 / thrust;

    attitude_rate = R.transpose() * (
        -Kr * Rde3.cross(R.col(2)) - (mass / thrust) * Rde3.cross((Eigen::Matrix3d::Identity() - Rde3 * Rde3.transpose()) * jdes[drone_id-1])
    ) / 3.141592 * 180;

    attitude[0] = atan2(-Rde3[1], Rde3[2]) / 3.141592 * 180;
    attitude[1] = asin(Rde3[0]) / 3.141592 * 180;
    attitude[2] = 0 / 3.141592 * 180;

    // this->controller_->set_attitude_rate(attitude_rate, thrust, dt);
    this->controller_->set_attitude(attitude, thrust, dt);

    // Set the attitude control message
    desired_attitude_msg_.attitude[0] = attitude[0];
    desired_attitude_msg_.attitude[1] = attitude[1];
    desired_attitude_msg_.attitude[2] = attitude[2];
    desired_attitude_msg_.thrust = thrust;

    // Publish the attitude control message for the controller to track
    desired_attitude_publisher_->publish(desired_attitude_msg_);

    // Set the attitude rate control message
    desired_attitude_rate_msg_.attitude[0] = attitude_rate[0];
    desired_attitude_rate_msg_.attitude[1] = attitude_rate[1];
    desired_attitude_rate_msg_.attitude[2] = attitude_rate[2];
    desired_attitude_rate_msg_.thrust = thrust;

    // Publish the position control message for the controller to track
    desired_attitude_rate_publisher_->publish(desired_attitude_rate_msg_);

    // Set the attitude rate control message
    desired_position_msg_.position[0] = pdes[drone_id-1][0];
    desired_position_msg_.position[1] = pdes[drone_id-1][1];
    desired_position_msg_.position[2] = pdes[drone_id-1][2];
    // desired_position_msg_.yaw = thrust;

    // Publish the attitude rate control message for the controller to track
    desired_position_publisher_->publish(desired_position_msg_);
}


bool BpeMode::exit() {
    return true;
}

void BpeMode::target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int id) {

    // Update the position of the other targets
    P_other[id] = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void BpeMode::update_vehicle_state() {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Update the MPC state
    P = state.position;
    V = state.velocity;
    Eigen::Quaterniond q(state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z());
    q.normalize();
    R = q.toRotationMatrix();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode, autopilot::Mode)