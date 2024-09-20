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

    // Load the gains of the controller
    node_->declare_parameter<double>("autopilot.BpeMode.gains.Kp", 2.0);
    node_->declare_parameter<double>("autopilot.BpeMode.gains.Kv", 0.5);
    node_->declare_parameter<double>("autopilot.BpeMode.gains.Kr", 10.0);
    Kp = node_->get_parameter("autopilot.BpeMode.gains.Kp").as_double();
    Kv = node_->get_parameter("autopilot.BpeMode.gains.Kv").as_double();
    Kr = node_->get_parameter("autopilot.BpeMode.gains.Kr").as_double();

    mass = get_vehicle_constants().mass;

    for (size_t i = 0; i < n_agents; i++) {
        pdes[i] = Eigen::Vector3d(0.0, 0.0, -2.0); // TODO verify that this should be negative
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
        omega = 1.0 - i*0.25;
        pdes[i][0] = A*sin(omega*t);
        pdes[i][1] = A*cos(omega*t);
        vdes[i][0] = omega*A*cos(omega*t);
        vdes[i][1] = -omega*A*sin(omega*t);
        udes[i][0] = -omega*omega*A*sin(omega*t);
        udes[i][1] = -omega*omega*A*cos(omega*t);
        jdes[i][0] = -omega*omega*omega*A*cos(omega*t);
        jdes[i][1] = omega*omega*omega*A*sin(omega*t);
    }

    if (drone_id==1) {
        u = udes[0] + Kp*(pdes[0] - P) + Kv*(vdes[0] - V);
    } else {
        u = udes[drone_id-1] - Kv * (V - vdes[drone_id-1]);
        for (size_t j = 0; j < n_agents; j++)
        {
            if (aij[drone_id-1][j])
            {
                pij = P_other[j] - P; // TODO how to update P_other???
                gij = pij.normalized();
                pijd = pdes[j] - pdes[drone_id-1];
                u = u - Kp*(pijd - gij * gij.dot(pijd));
            }   
        }  
    }

    TRde3 = -mass*g*e3 + mass*u;

    thrust = TRde3.norm();
    Rde3 = TRde3 / thrust;

    attitude_rate = R.transpose() * (
        -Kr * Rde3.cross(R.col(2)) - (mass / thrust) * Rde3.cross((Eigen::Matrix3d::Identity() - Rde3 * Rde3.transpose()) * jdes[drone_id-1])
    );

    this->controller_->set_attitude_rate(attitude_rate, thrust, dt); // TODO in rad/s or degrees/s, and roll, pitch, yaw or Z-Y-X?
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