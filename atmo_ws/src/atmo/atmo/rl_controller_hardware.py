# Numpy imports
from numpy import array,zeros

# ROS imports
import rclpy
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import InputRc
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorMotors

# Morphing lander imports
from atmo.mpc.RLBase    import RLBase
from atmo.mpc.parameters_rl import params_

min              = params_.get('min')
max              = params_.get('max')
dead             = params_.get('dead')
offboard_channel = params_.get('offboard_channel')
rl_channel       = params_.get('rl_channel')
kill_channel     = params_.get('kill_channel')

class RLHardware(RLBase): 
    def __init__(self):
        super().__init__()

        self.rc_subscription = self.create_subscription(
                                    InputRc,
                                    '/fmu/out/input_rc',
                                    self.rc_listener_callback,
                                    qos_profile_sensor_data)
        self.rc_subscription         
        self.vehicle_odometry_subscriber = self.create_subscription(
                                            VehicleOdometry,
                                            '/fmu/out/vehicle_odometry',
                                            self.vehicle_odometry_callback,
                                            qos_profile_sensor_data)
        self.vehicle_odometry_subscriber  

        self.min             = min
        self.max             = max
        self.dead            = dead
        self.offboard_switch = False
        self.rl_switch       = False
        self.kill_switch     = False

    def rl_trigger(self):
        return self.rl_switch
        
    def offboard_mode_trigger(self):
        return self.offboard_switch
    
    def vehicle_odometry_callback(self, msg): 
        self.position         = msg.position
        self.quaternion       = msg.q
        self.velocity         = msg.velocity
        self.angular_velocity = msg.angular_velocity
        print(f"position: {self.position}")
        print(f"quaternion: {self.quaternion}")
        print(f"velocity: {self.velocity}")
        print(f"angular_velocity: {self.angular_velocity}")
        

    def rc_listener_callback(self, msg):
        if (msg.values[offboard_channel] == self.max):
            self.offboard_switch = True
        else:
            self.offboard_switch = False

        if (msg.values[rl_channel] == self.max):
            self.rl_switch = True
        else:
            self.rl_switch = False

        if (msg.values[kill_channel] == self.max):
            self.kill_switch = True
        else:
            self.kill_switch = False

    def publish_actuator_motors(self,u):
        # publish to px4
        print(f"publishing: {u}")
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]
        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.actuator_motors_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Spinning RLHardware node... \n")
    rl_hardware = RLHardware()
    rclpy.spin(rl_hardware)
    rl_hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
