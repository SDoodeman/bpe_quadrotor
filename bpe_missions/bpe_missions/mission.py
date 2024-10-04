import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import Waypoint, AddCircle, SetMode
from pegasus_msgs.msg import AutopilotStatus
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class Drone(Node):

    def __init__(self, id):
        super().__init__('drone_api_' + str(id))

        self.id = id
        self.namespace = 'drone'

        # Create the service clients for the drone
        self.add_waypoint_srv = self.create_client(Waypoint, '/drone' + str(id) +'/autopilot/set_waypoint')
        print('Initializing service: /drone' + str(id) +'/autopilot/set_waypoint')
        while not self.add_waypoint_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not availtimeout_secable, waiting again...')

        self.add_circle_srv = self.create_client(AddCircle, '/drone' + str(id) +'/autopilot/trajectory/add_circle')
        print('Initializing service: /drone' + str(id) +'/autopilot/add_circle')
        while not self.add_circle_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')

        self.set_autopilot_srv = self.create_client(SetMode, '/drone' + str(id) +'/autopilot/change_mode')
        print('Initializing service: /drone' + str(id) +'/autopilot/change_mode')
        while not self.set_autopilot_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')

        # Create subscriptions
        self.create_subscription(AutopilotStatus, '/drone' + str(id) + '/autopilot/status', self.autopilot_status_callback, qos_profile_sensor_data)

        # Requests messages
        self.waypoint_req = Waypoint.Request()
        self.circle_req = AddCircle.Request()
        self.set_mode_req = SetMode.Request()


    def autopilot_status_callback(self, msg):
        self.get_logger().info('Received autopilot status: %s' % msg.mode)

    def set_autopilot_mode(self, mode='DisarmMode'):

        self.get_logger().info('Setting autopilot mode to: %s' % mode)
            
        # Set the mode request
        self.set_mode_req.mode = mode
        
        # Make an async request
        self.future = self.set_autopilot_srv.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_waypoint(self, x, y, z, yaw):
        self.get_logger().info(f'Setting waypoint to: {x}, {y}, {z}, {yaw}')
        self.waypoint_req.position[0] = x
        self.waypoint_req.position[1] = y
        self.waypoint_req.position[2] = z
        self.waypoint_req.yaw = yaw
        
        # Make an async request
        self.future = self.add_waypoint_srv.call_async(self.waypoint_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    drones = []
    n_drones = 3
    for i in range(n_drones):
        drones.append(Drone(i+1))

    # Wait until initial position is received
    # for i in range(n_drones):
    #     while not drones[i].initial_position_received:
    #         rclpy.spin_once(drones[i])

    # Arm the drone
    for i in range(n_drones):
        drones[i].set_autopilot_mode('ArmMode')

    time.sleep(2)

    for i in range(n_drones):
        drones[i].set_autopilot_mode('TakeoffMode')

    # Wait for takeoff
    time.sleep(7)

    for i in range(n_drones):
        drones[i].set_autopilot_mode('BpeMode')

    # Land the drone
    time.sleep(600)
    for i in range(n_drones):
        drones[i].set_autopilot_mode('OnboardLandMode')

    # Shutdown the demo
    for i in range(n_drones):
        drones[i].destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()