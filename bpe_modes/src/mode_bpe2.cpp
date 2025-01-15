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
    aij_ << 0, 0, 0,
            1, 0, 1,
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

    // Initialize the desired trajectory
    initialize_trajectory();
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
        u = udes_[graph_ids_[drone_id_]];

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

    //this->controller_->set_position(pdes_[id], 0.0, dt);
    this->controller_->set_attitude(attitude, thrust, dt);

    // Update the total time ellapsed
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

    // Set the desired velocity of the leader along the x-axis
    double a = 2;
    double s = 0.3;
    double o = 16;
    double k = 1.3;
    double v_leader = 1/a*std::pow(k, (-s*std::pow((total_time_-o)/a, 2)));
    
    // Set the desired trajectory for the leader drone (only moves along the x-axis)
    pdes_[0][0] += v_leader*dt;

    // Define the current radius for the y axis
    a = 0.7;
    s = 0.2;
    o = 15.4;
    k = 1.3;
    double radius_y = 2.0 - 1/a*std::pow(k,(-s*std::pow((total_time_-o)/a, 2)));

    // Define the current radius for the y axis
    a = 2.7;
    s = 0.2;
    o = 15.4;
    k = 1.3;
    double radius_x = 0.7 - 1/a*std::pow(k,(-s*std::pow((total_time_-o)/a, 2)));

    // Define the desired position for the first follower
    pdes_[1][0] = pdes_[0][0] - 0.20;
    pdes_[1][1] = radius_y * cos(total_time_);
    pdes_[1][2] = pdes_[0][2] + radius_x * sin(total_time_);

    // Define the desired position for the second follower
    pdes_[2][0] = pdes_[0][0] + 0.20;
    pdes_[2][1] = radius_y * cos(-total_time_);
    pdes_[2][2] = pdes_[0][2] + radius_x * sin(-total_time_);

    // Define the desired velocity for the followers


    // Define the desired acceleration for the followers


    // Define the desired jerk for the followers
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode2, autopilot::Mode)