#include "pegasus_utils/rotations.hpp"
#include "bpe_modes/mode_bpe2.hpp"

namespace autopilot {

BpeMode2::~BpeMode2() {}

void BpeMode2::initialize() {

    // Getting the drone ID and subscribing to the state of the other drones on the network
    drone_id_ = get_vehicle_constants().id;

    // Get the mass of the vehicle
    mass_ = get_vehicle_constants().mass;

    // Configure the adjacency matrix
    // aij_ << 0, 0, 0,
    //         1, 0, 1,
    //         1, 1, 0;

    aij_ << 0, 0, 0,
            1, 0, 0,
            1, 1, 0;

    // Get the list of drone ids associated with the leader, the first follower and the second follower
    node_->declare_parameter<std::vector<int>>("autopilot.BpeMode2.drone_ids", std::vector<int>());
    std::copy_n(node_->get_parameter("autopilot.BpeMode2.drone_ids").as_integer_array().begin(), n_agents_, drone_ids_.begin());

    for (size_t i = 0; i < n_agents_; i++) {
        graph_ids_[drone_ids_[i]] = i;
    }

    // Update the number of drones that this drone is tracking, based on the adjacency matrix
    for (int j = 0; j < n_agents_; j++) {
        if (aij_(graph_ids_[drone_id_],j)) {
            N_following_ += 1;
        }
    }

    // ----------------------------------------------------------------
    // Initialize the gains
    // ----------------------------------------------------------------
    // Check if this drone is the leader or just a follower
    if (drone_id_ == drone_ids_[0]) {
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.leader.Kp", 0.0);
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.leader.Kv", 0.0);
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.leader.Kr", 0.0);

        Kp_ = node_->get_parameter("autopilot.BpeMode2.gains.leader.Kp").as_double();
        Kv_ = node_->get_parameter("autopilot.BpeMode2.gains.leader.Kv").as_double();
        Kr_ = node_->get_parameter("autopilot.BpeMode2.gains.leader.Kr").as_double();
        
    } else {
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.followers.Kp", 0.0);
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.followers.Kv", 0.0);
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.followers.Kr", 0.0);
        node_->declare_parameter<double>("autopilot.BpeMode2.gains.followers.Ko", 0.0);

        Kp_ = node_->get_parameter("autopilot.BpeMode2.gains.followers.Kp").as_double();
        Kv_ = node_->get_parameter("autopilot.BpeMode2.gains.followers.Kv").as_double();
        Kr_ = node_->get_parameter("autopilot.BpeMode2.gains.followers.Kr").as_double();
    }


    // Initialize the desired trajectory
    initialize_trajectory();

    // Log the current parameters
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode2 Kp: %f", Kp_);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode2 Kv: %f", Kv_);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode2 Kr: %f", Kr_);
}

void BpeMode2::update(double dt) {
    
    // Get the current state of the vehicle
    update_vehicle_state();

    // Update the desired trajectory
    trajectory_generation(dt);

    // Compute the desired acceleration
    Eigen::Vector3d u;

    // Get the id of the drone in the graph 
    int id = graph_ids_[drone_id_];

    // Compute the desired acceleration
    if (id == 0) {
        // If the drone is the leader, just follow the desired trajectory (PD + feed-forward term)
        // The leader drone only follows the desired trajectory and does not have any connection to the other drones
        u = udes_[0] - Kp_*(P_[0] - pdes_[0]) - Kv_*(V_[0] - vdes_[0]);

    } else {

        // Get the desired feed-forward term
        u = udes_[id];

        // For each vehicle that we measure the bearing
        for (int j=0; j < n_agents_; j++) {
            // Check if the current drone sees the other drone
            if(aij_(id,j)) {

                // Get the bearing measurement
                Eigen::Vector3d pij = P_[j] - P_[id];
                Eigen::Vector3d gij = pij.normalized();

                // Get the desired relative position to that vehicle
                Eigen::Vector3d pijd = pdes_[j] - pdes_[id];

                // Substract to the acceleration the a correction term in the tangent space of S2
                u += - Kp_*(pijd - gij * gij.dot(pijd));

                // Compute the desired velocity error (for the trajectory to be executed)
                u += -Kv_ / N_following_ * ((V_[j] - V_[id]) - (vdes_[j] - vdes_[id]));

                // TODO: Check if it is (j - id) or (id - j)
            }
        }
    }

    // TODO: only for debugging
    u = udes_[id] - Kp_*(P_[id] - pdes_[id]) - Kv_*(V_[id] - vdes_[id]);

    // Compute the desired total force to apply
    const static Eigen::Vector3d e3(0, 0, 1);
    Eigen::Vector3d F = mass_*9.81*e3 - mass_*u;

    // Get the desired thrust along the desired Zb axis
    double thrust = F.norm();

    // Get the desired Zb axis
    Eigen::Vector3d Rde3 = F.normalized();

    // Compute attitude according to Z-Y-X convention (assuming -pi < roll < pi)
    double yaw_des = 0;
    Eigen::Vector3d attitude;
    attitude[0] = Pegasus::Rotations::rad_to_deg(asin(Rde3[0]*sin(yaw_des)-Rde3[1]*cos(yaw_des)));
    attitude[1] = Pegasus::Rotations::rad_to_deg(atan2(Rde3[0]*cos(yaw_des)+Rde3[1]*sin(yaw_des), Rde3[2]));
    attitude[2] = Pegasus::Rotations::rad_to_deg(yaw_des);

    this->controller_->set_attitude(attitude, thrust, dt);

    total_time_ += dt;
}

bool BpeMode2::enter() {

    // Reset the total time
    total_time_ = 0.0;
    return true;
}

bool BpeMode2::exit() {

    // Reset the total time
    total_time_ = 0.0;
    return true;
}

void BpeMode2::target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int drone_id) {

    // Update the position of the other targets (used later to compute the relative bearing measurements)
    P_[graph_ids_[drone_id]] = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    V_[graph_ids_[drone_id]] = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}

void BpeMode2::update_vehicle_state() {

    // Get the current state of the vehicle
    State state = this->get_vehicle_state();

    // Get the current position
    P_[graph_ids_[drone_id_]] = state.position;

    // Get the current velocity
    V_[graph_ids_[drone_id_]] = state.velocity;

    // Get the current attitude of the vehicle
    R = state.attitude.toRotationMatrix();
}

void BpeMode2::initialize_trajectory() {

    for(size_t i = 0; i < n_agents_; i++) {
        pdes_[i] = Eigen::Vector3d::Zero();
        vdes_[i] = Eigen::Vector3d::Zero();
        udes_[i] = Eigen::Vector3d::Zero();
        jdes_[i] = Eigen::Vector3d::Zero();
    }

    // Define the initial position for the leader drone to start at
    pdes_[0][0] = -3.0;
    pdes_[0][1] =  0.0;
    pdes_[0][2] = -1.2;
}

void BpeMode2::trajectory_generation(double dt) {

    double leader_velocity_x = 6.0 / 120.0;
    pdes_[0][0] += leader_velocity_x * dt;

    double radius_x = 0.7;
    double radius_y = 1.8;

    double f = 0.03;

    // Define the desired position for the first follower
    pdes_[1][0] = pdes_[0][0] - 0.20;
    pdes_[1][1] = radius_y * cos(2*M_PI*f*total_time_);
    pdes_[1][2] = pdes_[0][2] + radius_x * sin(2*M_PI*f*total_time_);

    // Define the desired velocity for the first follower
    vdes_[1][0] = leader_velocity_x;
    vdes_[1][1] = -radius_y * 2*M_PI*f*sin(2*M_PI*f*total_time_);
    vdes_[1][2] =  radius_x * 2*M_PI*f*cos(2*M_PI*f*total_time_);

    // Define the desired acceleration for the first follower
    udes_[1][0] = 0.0;
    udes_[1][1] = -radius_y * std::pow(2*M_PI*f,2)*cos(2*M_PI*f*total_time_);
    udes_[1][2] = -radius_x * std::pow(2*M_PI*f,2)*sin(2*M_PI*f*total_time_);

    // -------------------------------------------------------------------------------

    double offset = 2 * M_PI / 3;

    // Define the desired position of the second follower
    pdes_[2][0] = pdes_[0][0] + 0.20;
    pdes_[2][1] = radius_y * cos(2*M_PI*f*total_time_ + offset);
    pdes_[2][2] = pdes_[0][2] + radius_x * sin(2*M_PI*f*total_time_ + offset);

    // Define the desired velocity for the first follower
    vdes_[2][0] = leader_velocity_x;
    vdes_[2][1] = -radius_y * 2*M_PI*f*sin(2*M_PI*f*total_time_ + offset);
    vdes_[2][2] =  radius_x * 2*M_PI*f*cos(2*M_PI*f*total_time_ + offset);

    // Define the desired acceleration for the first follower
    udes_[2][0] = 0.0;
    udes_[2][1] = -radius_y * std::pow(2*M_PI*f,2)*cos(2*M_PI*f*total_time_ + offset);
    udes_[2][2] = -radius_x * std::pow(2*M_PI*f,2)*sin(2*M_PI*f*total_time_ + offset);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode2, autopilot::Mode)