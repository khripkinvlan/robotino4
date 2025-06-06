#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import sys

class OmniRobot:
    def __init__(self, x, y, diameter=0.45, sensor_range=0.4, sensor_count=9):
        self.x = x
        self.y = y
        self.diameter = diameter
        self.radius = diameter / 2
        self.sensor_range = sensor_range
        self.sensor_count = sensor_count
        self.sensors = np.linspace(0, 360, sensor_count, endpoint=False)
        self.speed_x = 0
        self.speed_y = 0
        self.dt = 0.025
    
class SimulationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.target_x = 1.7
        self.target_y = 1.0

        # Window configuration
        self.setWindowTitle("Robotino viz")
        self.setFixedSize(800, 800)  # Fixed size window

        # Initialize simulation objects
        self.robot = OmniRobot(0.25, 1.0)

        # Setup PyQtGraph
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        self.graphWidget.setXRange(-0.5, 2.5)
        self.graphWidget.setYRange(-0.5, 2.5)
        self.graphWidget.setAspectLocked(True)
        self.graphWidget.setBackground('w')
        # Disable mouse interactions (zooming/panning)
        self.graphWidget.setMouseEnabled(x=False, y=False)
        self.graphWidget.setMenuEnabled(False)  # Disable right-click menu

        self.target_click = True
        self.graphWidget.scene().sigMouseClicked.connect(self.mouse_clicked) # Mouse click callback

        # Create graphics items
        self.robot_item = pg.ScatterPlotItem(size=self.robot.diameter * 232,
                                             brush=pg.mkBrush('blue'),
                                             pen=pg.mkPen('k', width=1))
        self.robot_center_item = pg.ScatterPlotItem(size=4,
                                             brush=pg.mkBrush('black'),
                                             pen=pg.mkPen('k', width=1))
        self.target_item = pg.ScatterPlotItem(size= 0.12 * 232,
                                                brush=pg.mkBrush('green'),
                                                pen=pg.mkPen('k', width=1))

        # Add robot, target and obstacle
        self.graphWidget.addItem(self.robot_item)
        self.graphWidget.addItem(self.robot_center_item)
        self.graphWidget.addItem(self.target_item)

        # Sensor beams
        self.sensor_lines = []
        for _ in range(self.robot.sensor_count):
            line = pg.PlotCurveItem(pen=pg.mkPen('r', width=1))
            self.sensor_lines.append(line)
            self.graphWidget.addItem(line)
        
        self.readings = []
        for i in range(9):
            self.readings.append(0.41)

        # Sensor distance text
        self.sensor_text = pg.TextItem(text="", color='g', anchor=(0, 0))
        self.graphWidget.addItem(self.sensor_text)

        # Setup timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(20)  # interval

        # Initial update
        self.update_visuals()
    
    def update(self):
        self.update_visuals()

    def update_visuals(self):
        # Update robot and obstacle positions
        self.robot_item.setData([self.robot.x], [self.robot.y])
        self.robot_center_item.setData([self.robot.x], [self.robot.y])
        self.target_item.setData([self.target_x], [self.target_y])

        # Update sensor beams
        sensor_text = "Sensors: " + ", ".join(f"{dist:.2f}" for dist in self.readings)
        self.sensor_text.setText(sensor_text)
        self.sensor_text.setPos(0.05, 1.9)

        for i, (angle, dist) in enumerate(zip(self.robot.sensors, self.readings)):
            rad = np.radians(angle)
            sx = self.robot.x + self.robot.radius * np.cos(rad)
            sy = self.robot.y + self.robot.radius * np.sin(rad)
            ex = sx + dist * np.cos(rad)
            ey = sy + dist * np.sin(rad)
            self.sensor_lines[i].setData([sx, ex], [sy, ey])

    def mouse_clicked(self, event):
         if event.button() == QtCore.Qt.MouseButton.LeftButton:
            # Convert to scene coordinates
            mouse_point = self.graphWidget.plotItem.vb.mapSceneToView(event.scenePos())
            x, y = mouse_point.x(), mouse_point.y()
            if self.target_click:
                self.target_x = x
                self.target_y = y

class VizNode(Node):
    def __init__(self):
        super().__init__('robotino4_viz')

        # Target coordinate publisher
        self.target_pos_pub = self.create_publisher(Point, '/robotino4/target_coordinate', 10)

        # Proximity sensors subscriber
        self.sensors_pub = self.create_subscription(Float64MultiArray, '/robotino4/proximity_sensors', self.sensors_callback, 10)

        # Odometry subscriber
        self.odom_pub = self.create_subscription(Odometry, '/robotino4/odometry', self.odo_callback, 10)
        
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = SimulationWindow()
        self.window.show()
        
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):

        # Prepare, publish and log target position data
        target_point_msg = Point()
        target_point_msg.x = self.window.target_x
        target_point_msg.y = self.window.target_y
        target_point_msg.z = 0.0
        self.target_pos_pub.publish(target_point_msg)

        # Process QT window (graphics)
        self.app.processEvents()
    
    def sensors_callback(self, msg):
        for i in range(9):
            self.window.readings[i] = msg.data[i]
    
    def odo_callback(self, msg):
        self.window.robot.x = msg.pose.pose.position.x
        self.window.robot.y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()