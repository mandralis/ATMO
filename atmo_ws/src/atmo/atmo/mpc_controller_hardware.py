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
from atmo.mpc.MPCBase    import MPCBase
from atmo.mpc.parameters import params_
from atmo.mpc.utils      import euler_from_quaternion

min            = params_.get('min')
max            = params_.get('max')
dead           = params_.get('dead')
max_dx         = params_.get('max_dx')
max_dy         = params_.get('max_dy')
max_dz         = params_.get('max_dz')
max_dpsi       = params_.get('max_dpsi')
offboard_channel = params_.get('offboard_channel')
mpc_channel      = params_.get('mpc_channel')
roll_channel     = params_.get('roll_channel')
pitch_channel    = params_.get('pitch_channel')
yaw_channel      = params_.get('yaw_channel')
throttle_channel = params_.get('throttle_channel')

class MPCHardware(MPCBase): 
    def __init__(self):
        super().__init__()

        # subscribers
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

        self.min  = min
        self.max  = max
        self.dead = dead

        self.offboard_switch = False
        self.mpc_switch      = False

    def mpc_trigger(self):
        return self.mpc_switch
        
    def offboard_mode_trigger(self):
        return self.offboard_switch
    
    def vehicle_odometry_callback(self, msg): 
        # get state from odometry
        p = msg.position
        phi,th,psi = euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity
        self.state = array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2]])
        self.state_rl = array([p[0],p[1],p[2],msg.q[1],msg.q[2],msg.q[3],msg.q[0],v[0],v[1],v[2],o[0],o[1],o[2]])

    def rc_listener_callback(self, msg):
        if (msg.values[offboard_channel] == self.max):
            self.offboard_switch = True
        else:
            self.offboard_switch = False

        if (msg.values[mpc_channel] == self.max):
            self.mpc_switch = True
        else:
            self.mpc_switch = False

        # get input for manual control
        self.input[0]  = max_dx   * -(msg.values[pitch_channel]-dead)/(max-dead)
        self.input[1]  = max_dy   * -(msg.values[roll_channel]-dead)/(max-dead)
        self.input[2]  = max_dz   * -(msg.values[throttle_channel]-dead)/(max-dead)
        self.input[3]  = max_dpsi * -(msg.values[yaw_channel]-dead)/(max-dead)

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
    print("Spinning MPCHardware node... \n")
    mpc_hardware = MPCHardware()
    rclpy.spin(mpc_hardware)
    mpc_hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
