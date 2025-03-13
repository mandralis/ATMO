# Abstract Base Class
from abc import ABC, abstractmethod

# ROS imports
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Message imports
from custom_msgs.msg import DriveVel

# Morphing Lander imports
from atmo.mpc.parameters import params_

Ts_drive_controller = params_['Ts_drive_controller']

class DriveControllerBase(Node,ABC): 
    def __init__(self):
        super().__init__('DriveControllerBase')

        # subscriptions
        self.drive_vel_subscription = self.create_subscription(
            DriveVel,
            '/drive_vel',
            self.drive_vel_callback,
            qos_profile_sensor_data)
        self.drive_vel_subscription  # drive vel subscription 

        # timer
        self.timer_ = self.create_timer(Ts_drive_controller, self.timer_callback)

        # drive speed and turn speed
        self.drive_speed  = 0.0
        self.turn_speed   = 0.0

    # timer callback
    def timer_callback(self):
        # use drive_vel callback/rc_input to control wheel motors
        self.update() 

    # timer methods
    @abstractmethod
    def update(self):
        pass
    
    # subscription callbacks
    def drive_vel_callback(self, msg):
        self.drive_speed = msg.drivespeed
        self.turn_speed  = msg.turnspeed
        