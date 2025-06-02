#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

import socket
import requests
import sys


# Robotino's connection info
IP_ADDRESS = '192.168.0.1'  # Local Robotino IP address
PORT = 80  # Port to connect to

class RobotinoTCPBridge(Node):
    def __init__(self):
        super().__init__('tcp_bridge', namespace='robotino4')

        # Declare start coordinates with default float values
        self.declare_parameter('robot_x_init', 0.0)  # default X position
        self.declare_parameter('robot_y_init', 0.0)  # default Y position

        self.robot_init_x = self.get_parameter('robot_x_init').value
        self.robot_init_y = self.get_parameter('robot_y_init').value
        
        # Initialize network components
        self.socket = self.connect_to_robotino()

        if self.socket is None:
            self.get_logger().fatal("Failed to connect to Robotino! Shutting down...")
            self.destroy_node()  # Cleanup ROS resources
            rclpy.shutdown()    # Terminate ROS context
            sys.exit(1)         # Exit with error code
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/robotino4/odometry', 10)

        # Proximity sensors publisher
        self.sensors_pub = self.create_publisher(Float64MultiArray, '/robotino4/proximity_sensors', 10)
        
        self.sensors_data_msg = Float64MultiArray()
        self.sensors_data_msg.data = [0.0] * 9  # Initialize with zeros

        # Velocity command subscriber
        self.twist_sub = self.create_subscription(Twist, '/robotino4/cmd_vel', self.cmdvel_callback, 10)

        # Velocities buffer
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.position_init_flag = False
        self.x_offset = 0.0
        self.y_offset = 0.0

        # Timer for periodic operations
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info("RobotinoOdometry node started")

    def cmdvel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y

    def timer_callback(self):
        # Pack and publish odometry data
        odometry_readings = self.get_odometry()
        for i in range(len(odometry_readings)):
            odometry_readings[i] = odometry_readings[i] * 1000
        self.publish_odometry(odometry_readings)

        # Pack and publish proximity sensors data
        sensors_readings = self.get_proximity_sensor_values()
        for i in range(len(sensors_readings)):
            sensors_readings[i] = sensors_readings[i] * 1000
        self.publish_sensors(sensors_readings)

        # Send buffered velocities
        self.send_velocity(self.vx/1000, self.vy/1000, self.omega)

    def publish_odometry(self, odometry_readings):
        odom_msg = Odometry()
        if not odometry_readings or len(odometry_readings) < 6:
            self.get_logger().warn("Incomplete odometry data received")
            return
        
        try:
            if (not self.position_init_flag):
                self.x_offset = self.robot_init_x - odometry_readings[0]
                self.y_offset = self.robot_init_y - odometry_readings[1]
                self.position_init_flag = True
            
            # Position
            odom_msg.pose.pose.position.x = float(odometry_readings[0]+self.x_offset)
            odom_msg.pose.pose.position.y = float(odometry_readings[1]+self.y_offset)
            # TODO: Orientation
            
            # Velocity
            odom_msg.twist.twist.linear.x = float(odometry_readings[3])
            odom_msg.twist.twist.linear.y = float(odometry_readings[4])
            odom_msg.twist.twist.angular.z = float(odometry_readings[5])
            
            # Timestamp
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Publish
            self.odom_pub.publish(odom_msg)
            self.get_logger().info(f"Published odometry: x = {odom_msg.pose.pose.position.x:5.4f}, y = {odom_msg.pose.pose.position.y:5.4f}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to pack odometry: {str(e)}")
            self.get_logger().debug(f"Problematic data: {odometry_readings}")
    
    def publish_sensors(self, sensors_readings):
        if not sensors_readings or len(sensors_readings) < 9:
            self.get_logger().warn("Incomplete sensors data received")
            return
        try:
            
            for i in range(9):
                self.sensors_data_msg.data[i] = max(sensors_readings[i]-0.025, 0.0)
            
            # Publish
            self.sensors_pub.publish(self.sensors_data_msg)
        
        except Exception as e:
            self.get_logger().error(f"Failed to pack sensors data: {str(e)}")
            self.get_logger().debug(f"Problematic data: {sensors_readings}")
    

    # Connect to Robotino
    def connect_to_robotino(self):
        try:
            # Create a TCP/IP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((IP_ADDRESS, PORT))
            self.get_logger().info("Successfully connected to Robotino!")
            return sock
        except Exception as e:
            self.get_logger().error("Error connecting to Robotino: {e}")
            return None

    # Get raw odometry from robotino
    def get_odometry(self):
        try:
            # Send HTTP GET request to retrieve proximity sensor values
            url = f"http://{IP_ADDRESS}/data/odometry"
            response = requests.get(url)

            # Check if the response is successful
            if response.status_code == 200:
                # Assuming the response returns an array of floats in a JSON format
                odometry_readings = response.json()  # Parse JSON response
                if len(odometry_readings) == 7:
                    return odometry_readings
                else:
                    self.get_logger().warn("Unexpected odometry data array length")
            else:
                self.get_logger().error("Error: Received status code {response.status_code}")

        except Exception as e:
            self.get_logger().error("Error retrieving odometry values: {e}")

        return None

    # Get the proximity sensors' values from robotino
    def get_proximity_sensor_values(self):
        try:
            # Send HTTP GET request to retrieve proximity sensor values
            url = f"http://{IP_ADDRESS}/data/distancesensorarray"
            response = requests.get(url)

            # Check if the response is successful
            if response.status_code == 200:
                # Assuming the response returns an array of floats in a JSON format
                sensor_values = response.json()  # Parse JSON response
                if len(sensor_values) == 9:
                    return sensor_values
                else:
                    self.get_logger().warn("Unexpected proximity sensors data array length")
            else:
                self.get_logger().error("Error: Received status code {response.status_code}")

        except Exception as e:
            self.get_logger().error("Error retrieving proximity sensors values: {e}")

        return None
    
    # Get the proximity sensors' values from robotino
    def get_bumper(self):
        try:
            # Send HTTP GET request to retrieve proximity sensor values
            url = f"http://{IP_ADDRESS}/data/bumper"
            response = requests.get(url)

            # Check if the response is successful
            if response.status_code == 200:
                # Assuming the response returns an array of floats in a JSON format
                bumper_flag = response.json()  # Parse JSON response
                if len(bumper_flag) == 1:
                    return bumper_flag
                else:
                    self.get_logger().warn("Unexpected bumper data array length")
            else:
                self.get_logger().error("Error: Received status code {response.status_code}")

        except Exception as e:
            self.get_logger().error("Error retrieving proximity sensors values: {e}")

        return None


    # Sending commands to Robotino
    def send_velocity(self, vx, vy, omega):
        url = f"http://{IP_ADDRESS}/data/omnidrive"
        data = [vx, vy, omega]  # Prepare the data as a list

        try:
            # Send the velocity data to Robotino
            response = requests.post(url, json=data) # Send data as JSON
            if response.status_code == 200:
                print(f"Sent Vx: {vx}, Vy: {vy}")
                self.get_logger().info(f"Sent command to Robotino --- Vx: {vx}, Vy: {vy}")
            else:
                self.get_logger().warn("Failed to send data: {response.status_code} - {response.text}")
        except Exception as e:
            self.get_logger().error("Error sending velocity command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotinoTCPBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()