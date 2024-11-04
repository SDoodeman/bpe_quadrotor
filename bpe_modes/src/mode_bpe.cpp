#include "pegasus_utils/rotations.hpp"
#include "bpe_modes/mode_bpe.hpp"

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
    
    // Configure the adjacency matrix
    aij[1][0] = 1;
    aij[2][1] = 1;
    aij[2][0] = 1;

    for (size_t j = 0; j < n_agents; j++) {
        if (aij[drone_id-leader_id][j]) {
            N_following += 1;
        }
    }

    r = 0.5; // Safety distance

    // --------------------------------------------------------------
    // Subscribe to the position of the other agents
    // --------------------------------------------------------------
    for (size_t i = 0; i < n_agents; i++) {
        if (aij[drone_id-leader_id][i] || (drone_id-leader_id == i)) {
            target_subs_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
                "/drone" + std::to_string(i+leader_id) + "/fmu/filter/state",
                rclcpp::SensorDataQoS(),
                [this, i](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    this->target_state_callback(msg, i);  // Capture i and pass to callback
                }
            ));
        }
    }

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
        node_->declare_parameter<double>("autopilot.BpeMode.gains.leader.Kp", 2.0);
        node_->declare_parameter<double>("autopilot.BpeMode.gains.leader.Kv", 0.5); // Above 5/6 is unstable
        node_->declare_parameter<double>("autopilot.BpeMode.gains.leader.Kr", 10.0); // Above 10 is unstable

        Kp = node_->get_parameter("autopilot.BpeMode.gains.leader.Kp").as_double();
        Kv = node_->get_parameter("autopilot.BpeMode.gains.leader.Kv").as_double();
        Kr = node_->get_parameter("autopilot.BpeMode.gains.leader.Kr").as_double();
    } else {

        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Kp", 2.0);
        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Kv", 0.5); // Not allowed to get below 1
        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Kr", 10.0); // Above 10 is unstable
        node_->declare_parameter<double>("autopilot.BpeMode.gains.followers.Ko", 1.0);

        Kp = node_->get_parameter("autopilot.BpeMode.gains.followers.Kp").as_double();
        Kv = node_->get_parameter("autopilot.BpeMode.gains.followers.Kv").as_double();
        Kr = node_->get_parameter("autopilot.BpeMode.gains.followers.Kr").as_double();
        Ko = node_->get_parameter("autopilot.BpeMode.gains.followers.Ko").as_double();
    }

    // --------------------------------------------------------------
    // Initialize the trajectory parameters
    // --------------------------------------------------------------
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.z", -2.0);
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.A_offset", 1.0);
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.frequency", 0.1);

    z_ = node_->get_parameter("autopilot.BpeMode.trajectory.z").as_double();
    A_offset_ = node_->get_parameter("autopilot.BpeMode.trajectory.A_offset").as_double();
    frequency_ = node_->get_parameter("autopilot.BpeMode.trajectory.frequency").as_double();

    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kp: %f", Kp);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kv: %f", Kv);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kr: %f", Kr);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Ko: %f", Ko);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode sim: " << sim);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode leader_id: " << leader_id);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode trajectory z: " << z_);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode trajectory A offset: " << A_offset_);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode trajectory frequency: " << frequency_);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode initialized");
}

bool BpeMode::enter() {

    // Reset the time
    t = 0;
    return true;
}

void BpeMode::update(double dt) {
    double fB;

    // Update the current total time (used to get the desired bearing from the trajectory)
    // (this works under the assumption that this mode is activated for all robots at the same time)
    t += dt;
    
    // Get the current state of the vehicle
    update_vehicle_state();

    // Get the desired trajectory for each agent
    update_desired_trajectory();

    // Compute the desired acceleration
    if (drone_id==leader_id) {

        // If the drone is the leader, just follow the desired trajectory
        u = udes[0] - Kp*(P - pdes[0]) - Kv*(V - vdes[0]);

    } else {

        // Compute the feed-forward term
        u = udes[drone_id-leader_id];

        // For each vehicle that we measure the bearing
        for (size_t j = 0; j < n_agents; j++) {
            if (aij[drone_id-leader_id][j]) {

                // Get the bearing measurement
                pij = P_other[j] - P;
                gij = pij.normalized();

                // Get the desired relative position to the leader
                pijd = pdes[j] - pdes[drone_id-leader_id];

                // Substract to the acceleration the a correction term in the tangent space of S2
                u += - Kp*(pijd - gij * gij.dot(pijd));

                // Compute the desired velocity error (for the trajectory to be executed)
                u += -Kv / N_following * ((V - V_other[j]) - (vdes[drone_id-leader_id] - vdes[j]));

                // Collission avoidance term (p_ij is -e_i from the paper)
                fB = gij.dot(V_other[j] - V) / (pij.norm() - r);
                u += Ko*gij*fB;
            }
        }
    }

    // Compute the desired force to apply
    TRde3 = mass*9.81*e3 - mass*u;

    // Get the desired thrust along the desired Zb axis
    thrust = TRde3.norm();

    // Get the desired Zb axis
    Rde3 = TRde3 / thrust;

    // Compute the desired attitude-rate in case of attitude-rate control
    attitude_rate = R.transpose() * (-Kr * Rde3.cross(R.col(2)) - (mass / thrust) * Rde3.cross((Eigen::Matrix3d::Identity() - Rde3 * Rde3.transpose()) * jdes[drone_id-leader_id]));

    // Convert the desired attitude-rate to degrees
    attitude_rate = Pegasus::Rotations::rad_to_deg(attitude_rate);

    // Compute attitude according to Z-Y-X convention
    double yaw_des = -2*M_PI/frequency_*t;
    attitude[0] = Pegasus::Rotations::rad_to_deg(asin(Rde3[0]*sin(yaw_des)-Rde3[1]*cos(yaw_des)));
    attitude[1] = Pegasus::Rotations::rad_to_deg(atan2(Rde3[0]*cos(yaw_des)+Rde3[1]*sin(yaw_des), Rde3[2]));
    attitude[2] = Pegasus::Rotations::rad_to_deg(yaw_des);

    // this->controller_->set_attitude_rate(attitude_rate, thrust, dt);
    this->controller_->set_attitude(attitude, thrust, dt);

    // ---------------------------------------------------------------
    // Send desired references for statistics and plotting
    // ---------------------------------------------------------------
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
    desired_position_msg_.position[0] = pdes[drone_id-leader_id][0];
    desired_position_msg_.position[1] = pdes[drone_id-leader_id][1];
    desired_position_msg_.position[2] = pdes[drone_id-leader_id][2];

    // Publish the attitude rate control message for the controller to track
    desired_position_publisher_->publish(desired_position_msg_);
}


bool BpeMode::exit() {
    return true;
}

void BpeMode::update_desired_trajectory() {

    // Get the desired trajectory for each agent
    double A;
    double omega = M_PI * 2 * frequency_;

    for (size_t i = 0; i < n_agents; i++) {
        
        A = double((i+1)*A_offset_);

        // Compute the desired position
        pdes[i][0] = A*sin(omega*t - i*M_PI/2);
        pdes[i][1] = A*cos(omega*t - i*M_PI/2);
        pdes[i][2] = z_;

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
    V_other[id] = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    
    // Since in the simulation all the drones start at the same position, we add a small offset to the position between the vehicles
    if (sim) P_other[id][1] += id*3.0;
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
    R = state.attitude.toRotationMatrix();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode, autopilot::Mode)