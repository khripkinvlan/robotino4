#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Point, PoseWithCovariance
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time

# Weights of some rules subgroups
close_range_weight = 2.7
repulsion_weight = 2.7

# Fuzzy Control System Setup
def setup_fuzzy_control():
    sens_uni = np.arange(0, 0.42, 0.01)
    sensors = []

    # Correctly initialize all 9 sensors (sensor1 to sensor9)
    for i in range(1, 10):  # 1..9
        sensor = ctrl.Antecedent(sens_uni, f'sensor{i}')
        sensor['low'] = fuzz.trapmf(sens_uni, [0, 0, 0.12, 0.17])
        sensor['mid'] = fuzz.trapmf(sens_uni, [0.12, 0.17, 0.25, 0.3])
        sensor['high'] = fuzz.trapmf(sens_uni, [0.25, 0.3, 0.42, 0.42])
        sensors.append(sensor)

    # Target
    target_error_uni = np.arange(-2,2,0.01)
    target_error_x = ctrl.Antecedent(target_error_uni, 'x_err')
    target_error_y = ctrl.Antecedent(target_error_uni, 'y_err')

    for err_input in [target_error_x, target_error_y]:
        err_input['pos_high'] = fuzz.trapmf(target_error_uni, [0.25, 0.4, 2, 2])
        err_input['pos_mid'] = fuzz.trapmf(target_error_uni, [0.14, 0.19, 0.25, 0.3])
        err_input['pos_low'] = fuzz.trapmf(target_error_uni, [0, 0.08, 0.14, 0.19])
        err_input['zero'] = fuzz.trimf(target_error_uni, [-0.08, 0, 0.08])
        err_input['neg_low'] = fuzz.trapmf(target_error_uni, [-0.19, -0.14, -0.08, 0])
        err_input['neg_mid'] = fuzz.trapmf(target_error_uni, [-0.3, -0.25, -0.19, -0.14])
        err_input['neg_high'] = fuzz.trapmf(target_error_uni, [-2, -2, -0.4, -0.25])

    target_angle_uni = np.arange(-180, 180, 10)
    target_angle = ctrl.Antecedent(target_angle_uni, 'targ_ang')
    target_angle['s6'] = fuzz.trapmf(target_angle_uni, [-180, -180, -140, -120])
    target_angle['s7'] = fuzz.trapmf(target_angle_uni, [-160, -140, -100, -80])
    target_angle['s8'] = fuzz.trapmf(target_angle_uni, [-120, -100, -60, -40])
    target_angle['s9'] = fuzz.trapmf(target_angle_uni, [-80, -60, -20, 0])
    target_angle['s1'] = fuzz.trapmf(target_angle_uni, [-40, -20, 20, 40])
    target_angle['s2'] = fuzz.trapmf(target_angle_uni, [0, 20, 60, 80])
    target_angle['s3'] = fuzz.trapmf(target_angle_uni, [40, 60, 100, 120])
    target_angle['s4'] = fuzz.trapmf(target_angle_uni, [80, 100, 140, 160])
    target_angle['s5'] = fuzz.trapmf(target_angle_uni, [120, 140, 180, 180])

    target_dist_uni = np.arange(0, 3, 0.01)
    target_abs = ctrl.Antecedent(target_dist_uni, 'targ_abs')
    target_abs['low'] = fuzz.trapmf(target_dist_uni, [0, 0, 0.2, 0.3])
    target_abs['mid'] = fuzz.trapmf(target_dist_uni, [0.2, 0.3, 0.4, 0.5])
    target_abs['high'] = fuzz.trapmf(target_dist_uni, [0.4, 0.5, 2.5, 2.5])

    # Velocity outputs (-1 to 1 m/s)s
    vel_output_uni = np.arange(-0.25, 0.25, 0.001)
    vx_output = ctrl.Consequent(vel_output_uni, 'vx_output', 'centroid')
    vy_output = ctrl.Consequent(vel_output_uni, 'vy_output', 'centroid')

    # Membership functions for velocity
    for output in [vx_output, vy_output]:
        output['neg_high'] = fuzz.trapmf(vel_output_uni, [-0.225, -0.225, -0.2, -0.15])
        output['neg_mid'] = fuzz.trimf(vel_output_uni, [-0.2, -0.15, -0.075])
        output['neg_low'] = fuzz.trimf(vel_output_uni, [-0.15, -0.075, 0])
        output['zero'] = fuzz.trimf(vel_output_uni, [-0.075, 0, 0.075])
        output['pos_low'] = fuzz.trimf(vel_output_uni, [0, 0.075, 0.15])
        output['pos_mid'] = fuzz.trimf(vel_output_uni, [0.075, 0.15, 0.2])
        output['pos_high'] = fuzz.trapmf(vel_output_uni, [0.15, 0.2, 0.25, 0.25])

    # vx_output.view()
    # plt.show()

    path_ctrl = ctrl.ControlSystem([
        # exampel
        # ctrl.Rule(sensors[0]['low'], (vx_output['neg_high'], vy_output['zero'])),
        # ============ Target pursuit ================

        #zeroerror
        ctrl.Rule(target_error_x['zero'] & target_error_y['zero'], (vx_output['zero'], vy_output['zero'])),

        # Close range rules
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['zero'], (vx_output['pos_low']%close_range_weight, vy_output['zero']%close_range_weight)),
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['pos_low'], (vx_output['pos_low']%close_range_weight, vy_output['pos_low']%close_range_weight)),
        ctrl.Rule(target_error_x['zero'] & target_error_y['pos_low'], (vx_output['zero']%close_range_weight, vy_output['pos_low']%close_range_weight)),
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['pos_low'], (vx_output['neg_low']%close_range_weight, vy_output['pos_low']%close_range_weight)),
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['zero'], (vx_output['neg_low']%close_range_weight, vy_output['zero']%close_range_weight)),
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['neg_low'], (vx_output['neg_low']%close_range_weight, vy_output['neg_low']%close_range_weight)),
        ctrl.Rule(target_error_x['zero'] & target_error_y['neg_low'], (vx_output['zero']%close_range_weight, vy_output['neg_low']%close_range_weight)),
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['neg_low'], (vx_output['pos_low']%close_range_weight, vy_output['neg_low']%close_range_weight)),

        # Mid-range rules (sensor-dependent)
        # pos_mid, zero(sensor 1)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['zero'] & ~sensors[0]['low'],
                  (vx_output['pos_mid'], vy_output['zero'])),

        # pos_mid, neg_low (sensor 1 or 9)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['neg_low'] & (sensors[0]['mid'] | sensors[8]['mid']),
                  (vx_output['pos_mid'], vy_output['neg_low'])),
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['neg_low'] & (sensors[0]['high'] | sensors[8]['high']),
                  (vx_output['pos_mid'], vy_output['neg_low'])),

        # pos_mid, neg_mid (sensor 9)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['neg_mid'] & sensors[8]['mid'],
                  (vx_output['pos_mid'], vy_output['neg_mid'])),
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['neg_mid'] & sensors[8]['high'],
                  (vx_output['pos_mid'], vy_output['neg_mid'])),

        # pos_low, neg_mid (sensor 8 or 9)
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['neg_mid'] & (sensors[7]['mid'] | sensors[8]['mid']),
                  (vx_output['pos_low'], vy_output['neg_mid'])),
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['neg_mid'] & (sensors[7]['high'] | sensors[8]['high']),
                  (vx_output['pos_low'], vy_output['neg_mid'])),

        # zero, neg_mid (sensor 8)
        ctrl.Rule(target_error_x['zero'] & target_error_y['neg_mid'] & sensors[7]['mid'],
                  (vx_output['zero'], vy_output['neg_mid'])),
        ctrl.Rule(target_error_x['zero'] & target_error_y['neg_mid'] & sensors[7]['high'],
                  (vx_output['zero'], vy_output['neg_mid'])),

        # neg_low, neg_mid (sensor 7 or 8)
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['neg_mid'] & (sensors[6]['mid'] | sensors[7]['mid']),
                  (vx_output['neg_low'], vy_output['neg_mid'])),
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['neg_mid'] & (sensors[6]['high'] | sensors[7]['high']),
                  (vx_output['neg_low'], vy_output['neg_mid'])),

        # neg_mid, neg_mid (sensor 6 or 7)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['neg_mid'] & (sensors[5]['mid'] | sensors[6]['mid']),
                  (vx_output['neg_mid'], vy_output['neg_mid'])),
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['neg_mid'] & (sensors[5]['high'] | sensors[6]['high']),
                  (vx_output['neg_mid'], vy_output['neg_mid'])),

        # neg_mid, neg_low (sensor 6)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['neg_low'] & sensors[5]['mid'],
                  (vx_output['neg_mid'], vy_output['neg_low'])),
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['neg_low'] & sensors[5]['high'],
                  (vx_output['neg_mid'], vy_output['neg_low'])),

        # neg_mid, zero(sensor 5 or 6)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['zero'] & (sensors[4]['mid'] | sensors[5]['mid']),
                  (vx_output['neg_mid'], vy_output['zero'])),
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['zero'] & (sensors[4]['high'] | sensors[5]['high']),
                  (vx_output['neg_mid'], vy_output['zero'])),

        # neg_mid, pos_low (sensor 5)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['pos_low'] & sensors[4]['mid'],
                  (vx_output['neg_mid'], vy_output['pos_low'])),
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['pos_low'] & sensors[4]['high'],
                  (vx_output['neg_mid'], vy_output['pos_low'])),

        # neg_mid, pos_mid (sensor 4 or 5)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['pos_mid'] & (sensors[3]['mid'] | sensors[4]['mid']),
                  (vx_output['neg_mid'], vy_output['pos_mid'])),
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['pos_mid'] & (sensors[3]['high'] | sensors[4]['high']),
                  (vx_output['neg_mid'], vy_output['pos_mid'])),

        # neg_low, pos_mid (sensor 3 or 4)
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['pos_mid'] & (sensors[2]['mid'] | sensors[3]['mid']),
                  (vx_output['neg_low'], vy_output['pos_mid'])),
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['pos_mid'] & (sensors[2]['high'] | sensors[3]['high']),
                  (vx_output['neg_low'], vy_output['pos_mid'])),

        # zero, pos_mid (sensor 3)
        ctrl.Rule(target_error_x['zero'] & target_error_y['pos_mid'] & sensors[2]['mid'],
                  (vx_output['zero'], vy_output['pos_mid'])),
        ctrl.Rule(target_error_x['zero'] & target_error_y['pos_mid'] & sensors[2]['high'],
                  (vx_output['zero'], vy_output['pos_mid'])),

        # pos_low, pos_mid (sensor 2 or 3)
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['pos_mid'] & (sensors[1]['mid'] | sensors[2]['mid']),
                  (vx_output['pos_low'], vy_output['pos_mid'])),
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['pos_mid'] & (sensors[1]['high'] | sensors[2]['high']),
                  (vx_output['pos_low'], vy_output['pos_mid'])),

        # pos_mid, pos_mid (sensor 2)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['pos_mid'] & sensors[1]['mid'],
                  (vx_output['pos_mid'], vy_output['pos_mid'])),
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['pos_mid'] & sensors[1]['high'],
                  (vx_output['pos_mid'], vy_output['pos_mid'])),

        # pos_mid, pos_low (sensor 1 or 2)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['pos_low'] & (sensors[0]['mid'] | sensors[1]['mid']),
                  (vx_output['pos_mid'], vy_output['pos_low'])),
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['pos_low'] & (sensors[0]['high'] | sensors[1]['high']),
                  (vx_output['pos_mid'], vy_output['pos_low'])),

        # Far range
        # pos_high, zero(sensor 1, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['zero'] & sensors[0]['high'],
                  (vx_output['pos_high'], vy_output['zero'])),

        # pos_high, neg_low (sensor 1, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['neg_low'] & sensors[0]['high'],
                  (vx_output['pos_high'], vy_output['neg_low'])),

        # pos_high, neg_mid (sensor 1 or 9, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['neg_mid'] & (sensors[0]['high'] | sensors[8]['high']),
                  (vx_output['pos_high'], vy_output['neg_mid'])),

        # pos_high, neg_high (sensor 9, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['neg_high'] & sensors[8]['high'],
                  (vx_output['pos_high'], vy_output['neg_high'])),

        # pos_mid, neg_high (sensor 8 or 9, only high)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['neg_high'] & (sensors[7]['high'] | sensors[8]['high']),
                  (vx_output['pos_mid'], vy_output['neg_high'])),

        # pos_low, neg_high (sensor 8, only high)
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['neg_high'] & sensors[7]['high'],
                  (vx_output['pos_low'], vy_output['neg_high'])),

        # zero, neg_high (sensor 8, only high)
        ctrl.Rule(target_error_x['zero'] & target_error_y['neg_high'] & sensors[7]['high'],
                  (vx_output['zero'], vy_output['neg_high'])),

        # neg_low, neg_high (sensor 7 or 8, only high)
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['neg_high'] & (sensors[6]['high'] | sensors[7]['high']),
                  (vx_output['neg_low'], vy_output['neg_high'])),

        # neg_mid, neg_high (sensor 7, only high)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['neg_high'] & sensors[6]['high'],
                  (vx_output['neg_mid'], vy_output['neg_high'])),

        # neg_high, neg_high (sensor 6 or 7, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['neg_high'] & (sensors[5]['high'] | sensors[6]['high']),
                  (vx_output['neg_high'], vy_output['neg_high'])),

        # neg_high, neg_mid (sensor 6, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['neg_mid'] & sensors[5]['high'],
                  (vx_output['neg_high'], vy_output['neg_mid'])),

        # neg_high, neg_low (sensor 6, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['neg_low'] & sensors[5]['high'],
                  (vx_output['neg_high'], vy_output['neg_low'])),

        # neg_high, zero(sensor 5 or 6, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['zero'] & (sensors[4]['high'] | sensors[5]['high']),
                  (vx_output['neg_high'], vy_output['zero'])),

        # neg_high, pos_low (sensor 5, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['pos_low'] & sensors[4]['high'],
                  (vx_output['neg_high'], vy_output['pos_low'])),

        # neg_high, pos_mid (sensor 5, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['pos_mid'] & sensors[4]['high'],
                  (vx_output['neg_high'], vy_output['pos_mid'])),

        # neg_high, pos_high (sensor 4 or 5, only high)
        ctrl.Rule(target_error_x['neg_high'] & target_error_y['pos_high'] & (sensors[3]['high'] | sensors[4]['high']),
                  (vx_output['neg_high'], vy_output['pos_high'])),

        # neg_mid, pos_high (sensor 4, only high)
        ctrl.Rule(target_error_x['neg_mid'] & target_error_y['pos_high'] & sensors[3]['high'],
                  (vx_output['neg_mid'], vy_output['pos_high'])),

        # neg_low, pos_high (sensor 3 or 4, only high)
        ctrl.Rule(target_error_x['neg_low'] & target_error_y['pos_high'] & (sensors[2]['high'] | sensors[3]['high']),
                  (vx_output['neg_low'], vy_output['pos_high'])),

        # zero, pos_high (sensor 3, only high)
        ctrl.Rule(target_error_x['zero'] & target_error_y['pos_high'] & sensors[2]['high'],
                  (vx_output['zero'], vy_output['pos_high'])),

        # pos_low, pos_high (sensor 3, only high)
        ctrl.Rule(target_error_x['pos_low'] & target_error_y['pos_high'] & sensors[2]['high'],
                  (vx_output['pos_low'], vy_output['pos_high'])),

        # pos_mid, pos_high (sensor 2 or 3, only high)
        ctrl.Rule(target_error_x['pos_mid'] & target_error_y['pos_high'] & (sensors[1]['high'] | sensors[2]['high']),
                  (vx_output['pos_mid'], vy_output['pos_high'])),

        # pos_high, pos_high (sensor 2, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['pos_high'] & sensors[1]['high'],
                  (vx_output['pos_high'], vy_output['pos_high'])),

        # pos_high, pos_mid (sensor 1 or 2, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['pos_mid'] & (sensors[0]['high'] | sensors[1]['high']),
                  (vx_output['pos_high'], vy_output['pos_mid'])),

        # pos_high, pos_low (sensor 1, only high)
        ctrl.Rule(target_error_x['pos_high'] & target_error_y['pos_low'] & sensors[0]['high'],
                  (vx_output['pos_high'], vy_output['pos_low'])),

        # ============= OBSTACLE REPULSION ====================
        # Rule 1: Sensor 1 (Front)
        ctrl.Rule(sensors[0]['low'] & ~(target_error_x['pos_low'] & target_error_y['zero']) & ~(target_error_x['pos_low'] & target_error_y['neg_low']) & ~(target_error_x['pos_low'] & target_error_y['pos_low']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['neg_high']%repulsion_weight, vy_output['zero']%repulsion_weight)),

        # Rule 2: Sensor 2 (Front-left) - Avoid unless target is front-right (pos_low, pos_low)
        ctrl.Rule(sensors[1]['low'] & ~(target_error_x['pos_low'] & target_error_y['pos_low']) & ~(target_error_x['pos_low'] & target_error_y['zero']) & ~(target_error_x['zero'] & target_error_y['pos_low']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['neg_high']%repulsion_weight, vy_output['neg_high']%repulsion_weight)),

        # Rule 3: Sensor 3 (left) - Avoid unless target is right (zero, pos_low)
        ctrl.Rule(sensors[2]['low'] & ~(target_error_x['zero'] & target_error_y['pos_low']) & ~(target_error_x['pos_low'] & target_error_y['pos_low']) & ~(target_error_x['neg_low'] & target_error_y['pos_low']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['zero']%repulsion_weight, vy_output['neg_high']%repulsion_weight)),

        # Rule 4|5: Sensor 4 or 5 (Rear-left) - Avoid unless target is rear-right (neg_low, pos_low)
        ctrl.Rule((sensors[3]['low'] | sensors[4]['low']) & ~(target_error_x['neg_low'] & target_error_y['pos_low']) & ~(target_error_x['zero'] & target_error_y['pos_low']) & ~(target_error_x['neg_low'] & target_error_y['zero']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['pos_high']%repulsion_weight, vy_output['neg_high']%repulsion_weight)),

        # Rule 5|6: Sensor 5 or 6 (Rear) - Avoid unless target is rear (neg_low, zero)
        ctrl.Rule((sensors[4]['low'] | sensors[5]['low']) & ~(target_error_x['neg_low'] & target_error_y['zero']) & ~(target_error_x['neg_low'] & target_error_y['pos_low']) & ~(target_error_x['neg_low'] & target_error_y['neg_low']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['pos_high']%repulsion_weight, vy_output['zero']%repulsion_weight)),

        # Rule 6|7: Sensor 6 or 7 (Rear-right) - Avoid unless target is rear-left (neg_low, neg_low)
        ctrl.Rule((sensors[5]['low'] | sensors[6]['low']) & ~(target_error_x['neg_low'] & target_error_y['neg_low']) & ~(target_error_x['neg_low'] & target_error_y['zero']) & ~(target_error_x['zero'] & target_error_y['neg_low']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['pos_high']%repulsion_weight, vy_output['pos_high']%repulsion_weight)),

        # Rule 8: Sensor 8 (right) - Avoid unless target is left (zero, neg_low)
        ctrl.Rule(sensors[7]['low'] & ~(target_error_x['zero'] & target_error_y['neg_low']) & ~(target_error_x['neg_low'] & target_error_y['neg_low']) & ~(target_error_x['pos_low'] & target_error_y['neg_low']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['zero']%repulsion_weight, vy_output['pos_high']%repulsion_weight)),


        # Rule 9: Sensor 9 (Front-right) - Avoid unless target is front-left (pos_low, neg_low)
        ctrl.Rule(sensors[8]['low'] & ~(target_error_x['pos_low'] & target_error_y['neg_low']) & ~(target_error_x['zero'] & target_error_y['neg_low']) & ~(target_error_x['pos_low'] & target_error_y['zero']) & ~(target_error_x['zero'] & target_error_y['zero']),
                  (vx_output['neg_high']%repulsion_weight, vy_output['pos_high']%repulsion_weight)),
        
        # AVOIDANCE
        
    ])

    return ctrl.ControlSystemSimulation(path_ctrl)

class FuzzCtrlNode(Node):
    def __init__(self):
        super().__init__('robotino4_fuzzy_controller')

        # Velocity command publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/robotino4/cmd_vel', 10)
        
        # Proximity sensors subscription
        self.sensors_sub = self.create_subscription(Float64MultiArray, '/robotino4/proximity_sensors', self.sensors_callback, 10)

        # Robot coordinate publisher
        self.robot_odo_sub = self.create_subscription(Odometry, '/robotino4/odometry', self.robot_odo_callback, 10)

        # Target coordinate publisher
        self.target_pos_sub = self.create_subscription(Point, '/robotino4/target_coordinate', self.target_pos_callback, 10)

        # Create a fuzzy controller instance
        self.fuzzy_controller = setup_fuzzy_control()

        # ==== Create vars and initialize ====
        # Sensors readings buffer
        self.readings = []
        for i in range(9):
            self.readings.append(0.40)
        
        # Robot coordinates buffer
        self.robot_x = 0
        self.robot_y = 0

        # Target coordinates buffer
        self.target_x = 0
        self.target_y = 0

        # Data for calculating average compute time
        self.compute_counter = 0
        self.compute_time_sum = 0

        # Timer for periodic processes execution
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        start = time.time() # for compute time measurement

        # Input sensors data
        self.fuzzy_controller.input['sensor1'] = self.readings[0]
        self.fuzzy_controller.input['sensor2'] = self.readings[1]
        self.fuzzy_controller.input['sensor3'] = self.readings[2]
        self.fuzzy_controller.input['sensor4'] = self.readings[3]
        self.fuzzy_controller.input['sensor5'] = self.readings[4]
        self.fuzzy_controller.input['sensor6'] = self.readings[5]
        self.fuzzy_controller.input['sensor7'] = self.readings[6]
        self.fuzzy_controller.input['sensor8'] = self.readings[7]
        self.fuzzy_controller.input['sensor9'] = self.readings[8]

        # self.fuzzy_controller.input['targ_ang'] = self.target_ang
        # self.fuzzy_controller.input['targ_abs'] = self.target_abs

        self.fuzzy_controller.input['x_err'] = self.target_x-self.robot_x
        self.fuzzy_controller.input['y_err'] = self.target_y-self.robot_y

        # Compute outputs
        self.fuzzy_controller.compute()
        current_compute_time = time.time()-start
        
        # Log time in debug verbosity
        self.compute_counter += 1
        self.compute_time_sum += current_compute_time
        self.get_logger().info(
            f'Average compute time={(self.compute_time_sum/self.compute_counter):.6f}')
        self.get_logger().info(
            f'Current compute time={current_compute_time:.6f}')

        msg = Twist()
        # Set output values to a message
        try:
            msg.linear.x = self.fuzzy_controller.output['vx_output']
        except:
            msg.linear.x = 0.0
        
        try:
            msg.linear.y = self.fuzzy_controller.output['vy_output']
        except:
            msg.linear.y = 0.0
        
        # Publish and log
        self.cmdvel_publisher.publish(msg)
        self.get_logger().info(
            f'Publishing Twist: vx={msg.linear.x:5.4f}, vy={msg.linear.y:5.4f}')
    
    # Callback for sensors data recieve
    def sensors_callback(self, msg):
        for i in range(9):
            self.readings[i] = msg.data[i]
    
    # Callback for robot position point recieve
    def robot_odo_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
    
    # Callback for target position point receive
    def target_pos_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

def main(args=None):
    rclpy.init(args=args)
    node = FuzzCtrlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()