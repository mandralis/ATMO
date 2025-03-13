# Abstract Base Class
from abc import ABC, abstractmethod

# ROS imports
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import TiltAngle
from custom_msgs.msg import TiltVel

# Morphing Lander imports
from atmo.mpc.parameters import params_

Ts_tilt_controller = params_['Ts_tilt_controller']
queue_size         = params_['queue_size']
initial_tilt_sim   = params_.get('initial_tilt_sim')

class TiltControllerBase(Node,ABC): 
    def __init__(self):
        super().__init__('TiltControllerBase')

        # publishers
        self.publisher_tilt = self.create_publisher(TiltAngle, '/fmu/in/tilt_angle', queue_size)

        # subscriptions
        self.tilt_vel_subscription = self.create_subscription(
            TiltVel,
            '/tilt_vel',
            self.tilt_vel_callback,
            qos_profile_sensor_data)
        self.tilt_vel_subscription  # tilt vel subscription (value between -1 and 1)
 
        # timer
        self.Ts = Ts_tilt_controller 
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # tilt angle and tilt velocity
        self.tilt_angle = initial_tilt_sim
        self.tilt_vel   = 0.0

    # timer callback
    def timer_callback(self):
        # reset encoder if needed
        self.reset_encoder_trigger()
  
        # get current tilt angle 
        tilt_angle = self.get_current_tilt_angle()

        # publish current tilt angle
        self.publish_tilt_angle(tilt_angle)

        # use tilt_vel callback/rc_input to control tilt angle
        self.update() 

    # timer methods
    @abstractmethod
    def reset_encoder_trigger(self):
        pass

    @abstractmethod
    def get_current_tilt_angle(self):
        pass

    @abstractmethod
    def update(self):
        pass
    
    # subscription callbacks
    def tilt_vel_callback(self, msg):
        self.tilt_vel = msg.value # in rad/s

    # publisher methods
    def publish_tilt_angle(self,tilt_angle):
        msg = TiltAngle()
        msg.value = tilt_angle
        self.publisher_tilt.publish(msg)

