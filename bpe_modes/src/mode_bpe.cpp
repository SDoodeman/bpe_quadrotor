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
    node_->declare_parameter<int>("autopilot.BpeMode.first_drone_id", 1);
    first_drone_id = node_->get_parameter("autopilot.BpeMode.first_drone_id").as_int();
    // leader_id = first_drone_id;  // In case a real drone is the leader
    leader_id = first_drone_id - 1; // Virtual leader
    
    // Configure the adjacency matrix
    aij[1][0] = 1;
    aij[2][1] = 1;
    aij[2][0] = 1;
    aij[3][2] = 1;
    aij[3][1] = 1;
    aij[3][0] = 1;

    for (size_t j = 0; j < n_agents; j++) {
        if (aij[drone_id-leader_id][j]) {
            N_following += 1;
        }
    }

    //r = 0.5; // Safety distance

    // --------------------------------------------------------------
    // Subscribe to the position of the other real agents
    // --------------------------------------------------------------
    for (size_t i = first_drone_id-leader_id; i < n_agents; i++) {
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
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.z_min", -2.0);
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.z_max", -4.0);
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.A_offset", 1.0);
    node_->declare_parameter<double>("autopilot.BpeMode.trajectory.frequency", 0.1);
    node_->declare_parameter<double>("autopilot.BpeMode.avoidance.r", 0.1);

    r = node_->get_parameter("autopilot.BpeMode.avoidance.r").as_double();
    z_min = node_->get_parameter("autopilot.BpeMode.trajectory.z_min").as_double();
    z_max = node_->get_parameter("autopilot.BpeMode.trajectory.z_max").as_double();
    A_offset_ = node_->get_parameter("autopilot.BpeMode.trajectory.A_offset").as_double();
    frequency_ = node_->get_parameter("autopilot.BpeMode.trajectory.frequency").as_double();

    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kp: %f", Kp);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kv: %f", Kv);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Kr: %f", Kr);
    RCLCPP_INFO(this->node_->get_logger(), "BpeMode Ko: %f", Ko);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode sim: " << sim);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode first_drone_id: " << first_drone_id);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode trajectory z min: " << z_min);
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "BpeMode trajectory z max: " << z_max);
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

                // Collission avoidance term (ignore if other drone is virtual) (p_ij is -e_i from the paper)
                if (j < first_drone_id - leader_id) {
                    fB = gij.dot(V_other[j] - V) / (pij.norm() - r);
                    u += Ko*gij*fB;
                }
            }
        }
    }

    // Compute the desired force to apply
    TRde3 = mass*9.81*e3 - mass*u;

    // Prevent flipping the drone over
    // if (TRde3[2] < 0.0) {
    //     TRde3[2] = 0.0;
    // }

    // Get the desired thrust along the desired Zb axis
    thrust = TRde3.norm();

    // Get the desired Zb axis
    Rde3 = TRde3 / thrust;

    // Compute the desired attitude-rate in case of attitude-rate control
    attitude_rate = R.transpose() * (-Kr * Rde3.cross(R.col(2)) - (mass / thrust) * Rde3.cross((Eigen::Matrix3d::Identity() - Rde3 * Rde3.transpose()) * jdes[drone_id-leader_id]));

    // Convert the desired attitude-rate to degrees
    attitude_rate = Pegasus::Rotations::rad_to_deg(attitude_rate);

    // Compute attitude according to Z-Y-X convention (assuming -pi < roll < pi)
    double yaw_des = 0;
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
    double A, Adot, z, zdot;
    double omega = M_PI * 2 * frequency_;
    double t1{30};
    double t2{80};
    double t3{130};

    //double alpha = 0.15;

    double A_min = 0.40;

    for (size_t i = 0; i < n_agents; i++) {

        // This trajectory will start with wide circles (i*A_offset_) at a height of z_min.
        // From t=t1 to t=t2, the drones will start to move up and reduce the circle radius,
        //   which is at its smallest at t2 (half of the original), at which the height is
        //   in between z_min and z_max.
        // From t=t2 to t=t3, the drones continue to move up, but increase the circle radius,
        //   until the circles are at (i*A_offset_) again, at a height of z_max at t3.
        // After t3, the circles will stay at (i*A_offset_) at a height of z_max.

        /*
                   z_max  +              o- - - (z)
                          |             /
            (i*A_offset_) + - - -o     / o- - - (A)
                          |       \   / /
                          |        \ o /
                          |         \ / 
        0.5*(i*A_offset_) +        / o
                          |       /  
                    z_min + - - -o
                          +======|===|===|===========
                          t0     t1  t2  t3
        */                                   

        // Give virtual agents an amplitude of 10
        if (i < first_drone_id-leader_id) {
                A = 10;
                Adot = 0;
                z = (z_min + z_max) / 2;
                zdot = 0;
        } else {
            if (t < t1) {
                A = double(i)*A_offset_;
                Adot = 0;
                z = z_min;
                zdot = 0;
            } else if (t1 < t && t < t2) {
                //A = double(i)*A_offset_*(1 - alpha*(t - t1) / (t2 - t1));
                //Adot = -double(i)*A_offset_*alpha/(t2 - t1);
                A = double(i)*A_offset_ + ((A_min*double(i) - double(i)*A_offset_)*(t-t1)/(t2-t1));
                Adot = (A_min*double(i) - double(i)*A_offset_)/(t2-t1);
                z = z_min + (z_max - z_min) / 2 * (t - t1) / (t2 - t1);
                zdot = (z_max - z_min) / 2 / (t2 - t1);
            } else if (t2 < t && t < t3) {
                // A = double(i)*A_offset_*(alpha + alpha*(t - t2) / (t3 - t2));
                // Adot = double(i)*A_offset_*alpha/(t3 - t2);
                A = double(i)*A_min + ((double(i)*A_offset_ - A_min*double(i))*(t-t2)/(t3-t2));
                Adot = (double(i)*A_offset_ - A_min*double(i))/(t3-t2);
                z = (z_max + z_min) / 2 + (z_max - z_min) / 2 * (t - t2) / (t3 - t2);
                zdot = (z_max - z_min) / 2 / (t3 - t2);
            } else {
                A = double(i)*A_offset_;
                Adot = 0;
                z = z_max;
                zdot = 0;
            }
        }

        // Compute the desired position
        pdes[i][0] = A*sin(omega*t - i*M_PI/2);
        pdes[i][1] = A*cos(omega*t - i*M_PI/2);
        pdes[i][2] = z;

        // Compute the desired velocity
        vdes[i][0] =  omega*A*cos(omega*t - i*M_PI/2) + Adot*sin(omega*t - i*M_PI/2);
        vdes[i][1] = -omega*A*sin(omega*t - i*M_PI/2) + Adot*cos(omega*t - i*M_PI/2);
        vdes[i][2] = zdot;

        // Compute the desired acceleration
        udes[i][0] = -std::pow(omega,2)*A*sin(omega*t - i*M_PI/2) + 2*omega*Adot*cos(omega*t - i*M_PI/2);
        udes[i][1] = -std::pow(omega,2)*A*cos(omega*t - i*M_PI/2) - 2*omega*Adot*sin(omega*t - i*M_PI/2);
        udes[i][2] = 0.0;
        
        // Compute the desired jerk
        jdes[i][0] = -std::pow(omega,3)*A*cos(omega*t - i*M_PI/2) - 3*std::pow(omega,2)*Adot*sin(omega*t - i*M_PI/2);
        jdes[i][1] =  std::pow(omega,3)*A*sin(omega*t - i*M_PI/2) - 3*std::pow(omega,2)*Adot*cos(omega*t - i*M_PI/2);
        jdes[i][2] = 0.0;
    }

    // for (size_t i = 0; i < n_agents; i++) {
        
    //     A = double((i)*A_offset_);

    //     // Give virtual agents an amplitude of 10
    //     if (i < first_drone_id-leader_id) {
    //             A = 10;
    //     }

    //     // Compute the desired position
    //     pdes[i][0] = A*sin(omega*t - i*M_PI/2);
    //     pdes[i][1] = A*cos(omega*t - i*M_PI/2);
    //     pdes[i][2] = (z_min + z_max) / 2;

    //     // Compute the desired velocity
    //     vdes[i][0] =  omega*A*cos(omega*t - i*M_PI/2);
    //     vdes[i][1] = -omega*A*sin(omega*t - i*M_PI/2);
    //     vdes[i][2] = 0.0;

    //     // Compute the desired acceleration
    //     udes[i][0] = -std::pow(omega,2)*A*sin(omega*t - i*M_PI/2);
    //     udes[i][1] = -std::pow(omega,2)*A*cos(omega*t - i*M_PI/2);
    //     udes[i][2] = 0.0;
        
    //     // Compute the desired jerk
    //     jdes[i][0] = -std::pow(omega,3)*A*cos(omega*t - i*M_PI/2);
    //     jdes[i][1] =  std::pow(omega,3)*A*sin(omega*t - i*M_PI/2);
    //     jdes[i][2] = 0.0;
    // }

    // In case of a virtual leader
    for (size_t i = 0; i < first_drone_id - leader_id; i++) {
            P_other[i] = pdes[i];
            V_other[i] = vdes[i];
    }
}

void BpeMode::target_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int id) {

    // Update the position of the other targets (used later to compute the relative bearing measurements)
    P_other[id] = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    V_other[id] = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    
    // Since in the simulation all the drones start at the same position, we add a small offset to the position between the vehicles
    if (sim) P_other[id][1] += (id + leader_id - first_drone_id)*3.0;
}

void BpeMode::update_vehicle_state() {

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Get the current position
    P = state.position;

    // Since in the simulation all the drones start at the same position, we add a small offset to the position between the vehicles
    if (sim) P[1] += (drone_id-first_drone_id)*3.0;

    // Get the current velocity
    V = state.velocity;

    // Get the current attitude of the vehicle
    R = state.attitude.toRotationMatrix();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BpeMode, autopilot::Mode)