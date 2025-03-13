# Numpy imports
from numpy import pi, zeros

# ROS imports
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock

# Message imports
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAttitude

# Morphing lander imports
from atmo.mpc.RLBase        import RLBase
from atmo.mpc.parameters_rl import params_

warmup_time    = params_['warmup_time']
Ts             = params_['Ts']

class RLSim(RLBase): 
    def __init__(self):
        super().__init__()

        # subscribers
        self.vehicle_local_position_groundtruth = self.create_subscription(
                                            VehicleLocalPosition,
                                            '/fmu/out/vehicle_local_position_groundtruth',
                                            self.vehicle_local_position_groundtruth_callback,
                                            qos_profile_sensor_data)
        self.vehicle_local_position_groundtruth  # prevent unused variable warning

        self.vehicle_attitude_groundtruth = self.create_subscription(
                                            VehicleAttitude,
                                            '/fmu/out/vehicle_attitude_groundtruth',
                                            self.vehicle_attitude_groundtruth_callback,
                                            qos_profile_sensor_data)
        self.vehicle_attitude_groundtruth  # prevent unused variable warning

        self.vehicle_angular_velocity_groundtruth = self.create_subscription(
                                            VehicleAngularVelocity,
                                            '/fmu/out/vehicle_angular_velocity_groundtruth',
                                            self.vehicle_angular_velocity_groundtruth_callback,
                                            qos_profile_sensor_data)
        self.vehicle_angular_velocity_groundtruth  # prevent unused variable warning

    def rl_trigger(self):
        rl_flag = False
        self.counter += 1
        if (self.counter > int(warmup_time/Ts)): 
            rl_flag = True
        return rl_flag
        
    def offboard_mode_trigger(self):
        return True

    def publish_actuator_motors(self,u):
        # publish to px4
        print(f"publishing: {u}")
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]
        msg.control[4] = self.tilt_angle/(pi/2)
        msg.control[5] = self.tilt_angle/(pi/2)

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.actuator_motors_publisher_.publish(msg)

    def vehicle_local_position_groundtruth_callback(self, msg): 
        self.position[0] = msg.x
        self.position[1] = msg.y
        self.position[2] = msg.z
        self.velocity[0] = msg.vx
        self.velocity[1] = msg.vy
        self.velocity[2] = msg.vz

    def vehicle_attitude_groundtruth_callback(self, msg): 
        self.quaternion[0] = msg.q[0]
        self.quaternion[1] = msg.q[1]
        self.quaternion[2] = msg.q[2]
        self.quaternion[3] = msg.q[3]

    def vehicle_angular_velocity_groundtruth_callback(self, msg): 
        self.angular_velocity[0] = msg.xyz[0]
        self.angular_velocity[1] = msg.xyz[1]
        self.angular_velocity[2] = msg.xyz[2]

def main(args=None):
    rclpy.init(args=args)
    print("Spinning RLSim node \n")
    mpc_sim = RLSim()
    rclpy.spin(mpc_sim)
    mpc_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
