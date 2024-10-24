#include "pegasus_utils/rotations.hpp"
#include "bpe_modes/mode_bpe.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace autopilot {

BpeMode::~BpeMode() {}

void BpeMode::initialize() {
    
    // Getting the drone ID and subscribing to the state of the other drones on the network
    drone_id = get_vehicle_constants().id;

    // Load the parameter that checks if the experiment is simulation or real time
    node_->declare_parameter<bool>("autopilot.BpeMode.simulation", true);
    sim = node_->get_parameter("autopilot.BpeMode.simulation").as_bool();

    // Get the mass of the vehicle
    mass = get_vehicle_constants().mass;

    // --------------------------------------------------------------
    // Get the ID of the leader vehicle
    // --------------------------------------------------------------
    node_->declare_parameter<int>("autopilot.BpeMode.leader_id", 1);
    leader_id = node_->get_parameter("autopilot.BpeMode.leader_id").as_int();

    // --------------------------------------------------------------
    // Initialize the ROS 2 publisher for the statistics
    // --------------------------------------------------------------
    node_->declare_parameter<std::string>("autopilot.BpeController.publishers.control_attitude", "desired_control_attitude");
    node_->declare_parameter<std::string>("autopilot.BpeController.publishers.control_attitude_rate", "desired_control_attitude_rate");
    node_->declare_parameter<std::string>("autopilot.BpeController.publishers.control_position", "desired_control_position");

    // Create the publishers
    desired_attitude_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.BpeController.publishers.control_attitude").as_string(), rclcpp::SensorDataQoS());
    desired_attitude_rate_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.BpeController.publishers.control_attitude_rate").as_string(), rclcpp::SensorDataQoS());
    desired_position_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlPosition>(node_->get_parameter("autopilot.BpeController.publishers.control_position").as_string(), rclcpp::SensorDataQoS());

    // --------------------------------------------------------------
    // Initialize the gains
    // --------------------------------------------------------------
    if (drone_id == leader_id) {

        node_->declare_parameter<double>("autopilot.BpeMode.gains.leader.Kp", 10.0);
        node_->declare_parameter<double>("autopilot.BpeMode.gains.leader.Kv", 4.0);
        node_->declare_parameter<double>("autopilot.BpeMode.gains.leader.Kr", 10.0);

        Kp = node_->get_parameter("autopilot.BpeMode.gains.leader.Kp").as_double();
        Kv = node_->get_parameter("autopilot.BpeMode.gains.leader.Kv").as_double();
        Kr = node_->get_parameter("autopilot.BpeMode.gains.leader.Kr").as_double();

    } else {

        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Kp", 2.0);
        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Kv", 0.5);
        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Kr", 10.0);

        Kp = node_->get_parameter("autopilot.BpeMode.gains.followers.Kp").as_double();
        Kv = node_->get_parameter("autopilot.BpeMode.gains.followers.Kv").as_double();
        Kr = node_->get_parameter("autopilot.BpeMode.gains.followers.Kr").as_double();
    }

    // If running in simulation mode, subscribe to the Gazebo clock
    if (sim) {
        time_sub = node_->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 
            rclcpp::SensorDataQoS(),
            std::bind(&BpeMode::gz_clock_callback, this, std::placeholders::_1)
        );
    }

    // --------------------------------------------------------------
    // Subscribe to the position of the other agents
    // --------------------------------------------------------------
    for (size_t i = 0; i < n_agents; i++) {
        if (aij[drone_id-leader_id][i]) {
            target_subs_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
                "/drone" + std::to_string(i+leader_id) + "/fmu/filter/state",
                rclcpp::SensorDataQoS(),
                [this, i](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    this->target_state_callback(msg, i);  // Capture i and pass to callback
                }
            ));
        }
    }

    // Configure the adjacency matrix
    aij[1][0] = 1;
    aij[2][1] = 1;

    // Log the parameters of the controller
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode Kp: " << Kp);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode Kv: " << Kv);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode Kr: " << Kr);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode sim: " << sim);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode leader_id: " << leader_id);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode adjaceny matrix: " << aij);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode initialized");
}

bool BpeMode::enter() {

    return true;
}

void BpeMode::update(double dt) {
    
    // Get the current state of the vehicle
    update_vehicle_state();

    // Update the total-time using dt if running in real-time
    if (!sim) t += dt;

    // Get the desired trajectory for each agent
    update_desired_trajectory();

    // Compute the desired acceleration
    if (drone_id==leader_id) {
        // If the drone is the leader, just follow the desired trajectory
        u = udes[0] - Kp*(P - pdes[0]) - Kv*(V - vdes[0]);
    } else {
        // Otherwise, compute the desired acceleration using the BPE algorithm
        u = udes[drone_id-leader_id] - Kv * (V - vdes[drone_id-leader_id]);

        for (size_t j = 0; j < n_agents; j++) {
            if (aij[drone_id-leader_id][j]) {

                // Get the bearing measurement
                pij = P_other[j] - P;
                gij = pij.normalized();

                // Get the desired bearing
                pijd = pdes[j] - pdes[drone_id-leader_id];

                // Compute the desired acceleration witht the extra consensus term
                u = u - Kp*(pijd - gij * gij.dot(pijd));
            }
        }  
    }

    // Compute the desired force to apply
    TRde3 = mass*g*e3 - mass*u;

    // Get the desired thrust along the desired Zb axis
    thrust = TRde3.norm();

    // Get the desired Zb axis
    Rde3 = TRde3 / thrust;

    // Compute the desired attitude-rate
    attitude_rate = R.transpose() * (-Kr * Rde3.cross(R.col(2)) - (mass / thrust) * Rde3.cross((Eigen::Matrix3d::Identity() - Rde3 * Rde3.transpose()) * jdes[drone_id-leader_id]));

    // Convert the desired attitude-rate to degrees
    attitude_rate = Pegasus::Rotations::rad_to_deg(attitude_rate);

    this->controller_->set_attitude_rate(attitude_rate, thrust, dt);

    // ---------------------------------------------------------------
    // Compute the desired references for statistics and plotting
    // ---------------------------------------------------------------

    // Compute the desired attitude for statistics
    attitude[0] = Pegasus::Rotations::rad_to_deg(atan2(-Rde3[1], Rde3[2]));
    attitude[1] = Pegasus::Rotations::rad_to_deg(asin(Rde3[0]));
    attitude[2] = Pegasus::Rotations::rad_to_deg(0);

    // Set the attitude control message
    desired_attitude_msg_.attitude[0] = attitude[0];
    desired_attitude_msg_.attitude[1] = attitude[1];
    desired_attitude_msg_.attitude[2] = attitude[2];
    desired_attitude_msg_.thrust = thrust;

    //this->controller_->set_attitude(attitude, thrust, dt);

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
    desired_position_msg_.position[0] = pdes[drone_id-leader_id][0];
    desired_position_msg_.position[1] = pdes[drone_id-leader_id][1];
    desired_position_msg_.position[2] = pdes[drone_id-leader_id][2];

    // Publish the attitude rate control message for the controller to track
    desired_position_publisher_->publish(desired_position_msg_);
}


bool BpeMode::exit() {

    // Reset the time before exiting
    t = 0;
    return true;
}

void BpeMode::update_desired_trajectory() {

    double A;
    double omega = M_PI * 2 / 10;

    for (size_t i = 0; i < n_agents; i++) {
        
        A = double(i+1);

        // Compute the desired position
        pdes[i][0] = A*sin(omega*t - i*M_PI/2);
        pdes[i][1] = A*cos(omega*t - i*M_PI/2);
        pdes[i][2] = -2.0;

        // Compute the desired velocity
        vdes[i][0] =  omega*A*cos(omega*t - i*M_PI/2);
        vdes[i][1] = -omega*A*sin(omega*t - i*M_PI/2);
        vdes[i][2] = 0.0;

        // Compute the desired acceleration
        udes[i][0] = -std::pow(omega,2)*A*sin(omega*t - i*M_PI/2);
        udes[i][1] = -std::pow(omega,2)*A*cos(omega*t - i*M_PI/2);
        udes[i][2] = 0.0;
        
        // Compute the desired jerk
        jdes[i][0] = -std::pow(omega,3)*A*cos(omega*t - i*M_PI/2);
        jdes[i][1] =  std::pow(omega,3)*A*sin(omega*t - i*M_PI/2);
        jdes[i][2] = 0.0;
    }
}

void BpeMode::target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int id) {

    // Update the position of the other targets (used later to compute the relative bearing measurements)
    P_other[id] = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    
    // Since in the simulation all the drones start at the same position, we add a small offset to the position between the vehicles
    if (sim) P_other[id][1] += id*3.0;
}

void BpeMode::gz_clock_callback(const rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {

    // Get the current simulation time
    t = 1.0*msg->clock.sec + 1e-9*msg->clock.nanosec;
}

void BpeMode::update_vehicle_state() {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Get the current position
    P = state.position;

    // Since in the simulation all the drones start at the same position, we add a small offset to the position between the vehicles
    if (sim) P[1] += (drone_id-leader_id)*3.0;

    // Get the current velocity
    V = state.velocity;

    // Get the current attitude of the vehicle
    //Eigen::Quaterniond q(state.attitude.w(), state.attitude.x(), state.attitude.y(), state.attitude.z());
    //q.normalize();
    //R = q.toRotationMatrix();
    R = state.attitude.toRotationMatrix();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode, autopilot::Mode)