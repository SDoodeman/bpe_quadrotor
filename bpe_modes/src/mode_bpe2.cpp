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

    // --------------------------------------------------------------
    // Initialize the ROS 2 publisher for the statistics
    // --------------------------------------------------------------

    // Initialize the statistics publisher
    node_->declare_parameter<std::string>("autopilot.BpeMode2.statistics_publisher", "bpe_statistics");
    statistics_publisher_ = node_->create_publisher<bpe_msgs::msg::BpeStatistics>(
        node_->get_parameter("autopilot.BpeMode2.statistics_publisher").as_string(), rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("autopilot.BpeMode2.publishers.control_attitude", "desired_control_attitude");
    node_->declare_parameter<std::string>("autopilot.BpeMode2.publishers.control_attitude_rate", "desired_control_attitude_rate");
    node_->declare_parameter<std::string>("autopilot.BpeMode2.publishers.control_position", "desired_control_position");
    node_->declare_parameter<std::string>("autopilot.BpeMode2.publishers.position_error", "position_error");

    // Create the publishers
    desired_attitude_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.BpeMode2.publishers.control_attitude").as_string(), rclcpp::SensorDataQoS());
    desired_attitude_rate_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.BpeMode2.publishers.control_attitude_rate").as_string(), rclcpp::SensorDataQoS());
    desired_position_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlPosition>(node_->get_parameter("autopilot.BpeMode2.publishers.control_position").as_string(), rclcpp::SensorDataQoS());
    position_error_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(node_->get_parameter("autopilot.BpeMode2.publishers.position_error").as_string(), rclcpp::SensorDataQoS());
    total_time_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("total_time", rclcpp::SensorDataQoS());

    // --------------------------------------------------------------
    // Subscribe to the position of the other real agents
    // --------------------------------------------------------------
    for(int j=0; j < n_agents_; j++) {
        if (aij_(graph_ids_[drone_id_],j)) {
            target_subs_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
                "/drone" + std::to_string(drone_ids_[j]) + "/fmu/filter/state",
                rclcpp::SensorDataQoS(),
                [this, j](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    this->target_state_callback(msg, drone_ids_[j]);  // Capture i and pass to callback
                }
            ));
        }
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
                u += -Kv_ / N_following_ * ((V_[id] - V_[j]) - (vdes_[id] - vdes_[j]));
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

    this->controller_->set_attitude(attitude, thrust, dt);

    // Increment the total time elapsed
    total_time_ += dt;

    // --------------------------------
    // Set the statistics message
    // --------------------------------
    desired_position_msg_.position[0] = pdes_[id][0];
    desired_position_msg_.position[1] = pdes_[id][1];
    desired_position_msg_.position[2] = pdes_[id][2];
    desired_position_publisher_->publish(desired_position_msg_);

    position_error_msg_.data = (P_[id] - pdes_[id]).norm();
    position_error_publisher_->publish(position_error_msg_);

    total_time_msg_.data = total_time_;
    total_time_publisher_->publish(total_time_msg_);
}

bool BpeMode2::enter() {

    // Reset the total time
    total_time_ = 0.0;
    return true;
}

bool BpeMode2::exit() {

    // Reset the total time
    total_time_ = 0.0;
    initialize_trajectory();
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
}

void BpeMode2::trajectory_generation(double dt) {

    double z_min_ = -0.5;
    double z_max_ = -2.0;
    double A_offset_ = 0.55;
    double frequency_ = 0.06;

    // Get the desired trajectory for each agent
    double A, Adot, z, zdot;
    double omega = M_PI * 2 * frequency_;
    double t1{30};
    double t2{80};
    double t3{130};

    double t = total_time_;

    double A_min = 0.40;

    for (size_t i = 0; i < n_agents_; i++) {

        // This trajectory will start with wide circles (i*A_offset_) at a height of z_min.
        // From t=t1 to t=t2, the drones will start to move up and reduce the circle radius,
        //   which is at its smallest at t2 (half of the original), at which the height is
        //   in between z_min and z_max.
        // From t=t2 to t=t3, the drones continue to move up, but increase the circle radius,
        //   until the circles are at (i*A_offset_) again, at a height of z_max at t3.
        // After t3, the circles will stay at (i*A_offset_) at a height of z_max.

        // Trajectory for the leader drone
        if (i==0) {
                A = 1.5;
                Adot = 0;
                z = (z_min_ + z_max_) / 2;
                zdot = 0;
        // Trajectory for the follower drones
        } else {
            if (t < t1) {
                A = double(i)*A_offset_;
                Adot = 0;
                z = z_min_;
                zdot = 0;
            } else if (t1 < t && t < t2) {
                //A = double(i)*A_offset_*(1 - alpha*(t - t1) / (t2 - t1));
                //Adot = -double(i)*A_offset_*alpha/(t2 - t1);
                A = double(i)*A_offset_ + ((A_min*double(i) - double(i)*A_offset_)*(t-t1)/(t2-t1));
                Adot = (A_min*double(i) - double(i)*A_offset_)/(t2-t1);
                z = z_min_ + (z_max_ - z_min_) / 2 * (t - t1) / (t2 - t1);
                zdot = (z_max_ - z_min_) / 2 / (t2 - t1);
            } else if (t2 < t && t < t3) {
                // A = double(i)*A_offset_*(alpha + alpha*(t - t2) / (t3 - t2));
                // Adot = double(i)*A_offset_*alpha/(t3 - t2);
                A = double(i)*A_min + ((double(i)*A_offset_ - A_min*double(i))*(t-t2)/(t3-t2));
                Adot = (double(i)*A_offset_ - A_min*double(i))/(t3-t2);
                z = (z_max_ + z_min_) / 2 + (z_max_ - z_min_) / 2 * (t - t2) / (t3 - t2);
                zdot = (z_max_ - z_min_) / 2 / (t3 - t2);
            } else {
                A = double(i)*A_offset_;
                Adot = 0;
                z = z_max_;
                zdot = 0;
            }
        }

        // Compute the desired position
        pdes_[i][0] = A*sin(omega*t - i*M_PI/2);
        pdes_[i][1] = A*cos(omega*t - i*M_PI/2);
        pdes_[i][2] = z;

        // Compute the desired velocity
        vdes_[i][0] =  omega*A*cos(omega*t - i*M_PI/2) + Adot*sin(omega*t - i*M_PI/2);
        vdes_[i][1] = -omega*A*sin(omega*t - i*M_PI/2) + Adot*cos(omega*t - i*M_PI/2);
        vdes_[i][2] =  zdot;

        // Compute the desired acceleration
        udes_[i][0] = -std::pow(omega,2)*A*sin(omega*t - i*M_PI/2) + 2*omega*Adot*cos(omega*t - i*M_PI/2);
        udes_[i][1] = -std::pow(omega,2)*A*cos(omega*t - i*M_PI/2) - 2*omega*Adot*sin(omega*t - i*M_PI/2);
        udes_[i][2] =  0.0;
        
        // Compute the desired jerk
        jdes_[i][0] = -std::pow(omega,3)*A*cos(omega*t - i*M_PI/2) - 3*std::pow(omega,2)*Adot*sin(omega*t - i*M_PI/2);
        jdes_[i][1] =  std::pow(omega,3)*A*sin(omega*t - i*M_PI/2) - 3*std::pow(omega,2)*Adot*cos(omega*t - i*M_PI/2);
        jdes_[i][2] =  0.0;
    }
}} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode2, autopilot::Mode)