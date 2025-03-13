"""
Python implementation of Load Cell Test 

"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import TiltAngle
from px4_msgs.msg import InputRc
from px4_msgs.msg import ActuatorMotors

import numpy as np
from custom_msgs.msg import LoadCell
from atmo.mpc.utils import euler_from_quaternion

class LoadCellTest(Node):
    def __init__(self):
        super().__init__('load_cell_test')

        # Subscriptions
        self.rc_input_subscriber_ = self.create_subscription(
                                            InputRc,
                                            '/fmu/out/input_rc',
                                            self.rc_listener_callback,
                                            qos_profile_sensor_data)
        self.rc_input_subscriber_  # prevent unused variable warning

        self.tilt_angle_subscriber_ = self.create_subscription(
                                            TiltAngle,
                                            '/fmu/in/tilt_angle',
                                            self.tilt_angle_callback,
                                            qos_profile_sensor_data)
        self.tilt_angle_subscriber_  # prevent unused variable warning
        self.vehicle_odometry_subscriber = self.create_subscription(
                                            VehicleOdometry,
                                            '/fmu/out/vehicle_odometry',
                                            self.vehicle_odometry_callback,
                                            qos_profile_sensor_data)
        self.vehicle_odometry_subscriber  

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
                                                        OffboardControlMode, 
                                                        "/fmu/in/offboard_control_mode", 
                                                        10)
        self.vehicle_command_publisher_       = self.create_publisher(
                                                        VehicleCommand, 
                                                        "/fmu/in/vehicle_command", 
                                                        10)
        self.actuator_motors_publisher_       = self.create_publisher(
                                                        ActuatorMotors, 
                                                        "/fmu/in/actuator_motors", 
                                                        10)
        self.tilt_angle_ref_external_publisher   = self.create_publisher(
                                                        TiltAngle, 
                                                        "/tilt_angle_ref_external", 
                                                        10)
        self.load_cell_publisher_                = self.create_publisher(
                                                        LoadCell, 
                                                        "/load_cell", 
                                                        10)

        # Create timer
        self.offboard_setpoint_counter_ = 0
        self.Ts = 0.01  # 100 Hz
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set RC input limits
        self.min  = 1094
        self.dead = 1514
        self.max  = 1934

        # Desired tilt angle
        self.tilt_angle_desired  = 0.0

        # Current tilt_angle
        self.tilt_angle = 0.0

        # Altitude control logic
        self.switch_on = False

        # state
        self.state = np.zeros(12)

        # arming flag 
        self.offboard_flag = False
        self.offboard    = False 
        self.offboard_setpoint_counter_ = 0

        # Test throttle
        self.test_throttle = 0.20

    # usage: start in manual/stabilize mode and fly up. At that point flip the switch which will take you into offboard mode
    def timer_callback(self):

        # Go into offboard mode
        if (self.offboard_flag):
            # publish offboard mode heartbeat
            self.publish_offboard_control_mode()  

            # publish log
            self.publish_log(self.state)

            if not self.offboard:
                if (self.offboard_setpoint_counter_ == 10):
                    # go to offboard mode after 1 second
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

                    # Arm the vehicle
                    self.publish_actuator_motors(0.0)  # zero throttle if switch is not on
                    self.arm()

                    # vehicle is now in offboard mode and armed
                    self.offboard = True

                # stop the counter after reaching 11
                if (self.offboard_setpoint_counter_ < 11):
                    self.offboard_setpoint_counter_ += 1

        else:
            if self.offboard:
                self.disarm()  
                self.publish_actuator_motors(0.0)
                self.offboard = False 
                self.offboard_setpoint_counter_ = 0
        
        if self.offboard:
            if self.switch_on:
                self.publish_actuator_motors(self.test_throttle)

    def tilt_angle_callback(self, msg):
        self.tilt_angle = msg.value

    def rc_listener_callback(self, msg):
        # get arm command
        if (msg.values[8] == self.max):
            self.offboard_flag = True
        else:
            self.offboard_flag = False

        if (msg.values[7] == self.max):
            self.switch_on = True
        else:
            self.switch_on = False

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    # Give offboard control a heartbeat
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)
        
    def publish_actuator_motors(self, throttle):
        msg = ActuatorMotors()
        msg.control[0] = throttle
        msg.control[1] = throttle
        msg.control[2] = throttle
        msg.control[3] = throttle
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.actuator_motors_publisher_.publish(msg)   

    def publish_tilt_angle_ref(self, tilt_angle):
        msg = TiltAngle()
        msg.value = np.rad2deg(tilt_angle)
        self.tilt_angle_ref_external_publisher.publish(msg)

    def publish_log(self,x):
        msg = LoadCell()
        msg.x = x.astype('float32')
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.load_cell_publisher_.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def vehicle_odometry_callback(self, msg): 
        # get state from odometry
        p = msg.position
        phi,th,psi = euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity
        self.state = np.array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2]])


def main(args=None):
    rclpy.init(args=args)
    print("Starting load cell test...\n")
    load_cell_test = LoadCellTest()
    rclpy.spin(load_cell_test)
    load_cell_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
