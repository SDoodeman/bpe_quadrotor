#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import Waypoint, AddCircle, SetMode
from pegasus_msgs.msg import AutopilotStatus

import time



class Drone(Node):

    def __init__(self, id):

        super().__init__('drone_api_' + str('id'))

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

        self.get_logger().info('Setting waypoint to: %f, %f, %f, %f' % (x, y, z, yaw))
        
        # Set the waypoint request
        self.waypoint_req.position[0] = x
        self.waypoint_req.position[1] = y
        self.waypoint_req.position[2] = z
        self.waypoint_req.yaw = yaw
        
        # Make an async request
        self.future = self.add_waypoint_srv.call_async(self.waypoint_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_circle(self, center, radius, speed):

        self.get_logger().info('Setting circle to: %f, %f, %f, %f' % (center[0], center[1], center[2], radius))
        
        # Set the circle request
        self.circle_req.center[0] = center[0]
        self.circle_req.center[1] = center[1]
        self.circle_req.center[2] = center[2]
        self.circle_req.normal[0] = 0.0
        self.circle_req.normal[1] = 0.0
        self.circle_req.normal[2] = 1.0

        self.circle_req.radius = radius
        self.circle_req.speed.type = "constant"
        self.circle_req.speed.parameters = [speed]
        
        # Make an async request
        self.future = self.add_circle_srv.call_async(self.circle_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):

    # Initiate the ROS2 nodes
    rclpy.init(args=args)

    # Create the drones
    drones = [Drone(1), Drone(2), Drone(3)]

    height = -1.30

    for drone in drones:
        # Take off all of the drones
        drone.set_autopilot_mode('ArmMode')
        drone.set_autopilot_mode('TakeoffMode')

    # Wait for the drones to take off
    time.sleep(5)

    for drone in drones:
        # Set the bpe mode
        drone.set_autopilot_mode('BpeMode')

    # Wait for the drones to perform their mission
    time.sleep(60)

    # Land the drones
    for drone in drones:
        drone.set_autopilot_mode('LandMode')

    # Shutdown the demo
    for drone in drones:
        drone.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()