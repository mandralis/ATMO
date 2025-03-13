import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc

# roboclaw and jetson
from atmo.mpc.DriveControllerBase import DriveControllerBase
from atmo.mpc.roboclaw_3 import Roboclaw
from atmo.mpc.parameters import params_

# get parameters
min                    = params_.get('min')
max                    = params_.get('max')
dead                   = params_.get('dead')
Ts                     = params_.get('Ts_drive_controller')
drive_roboclaw_address = params_.get('drive_roboclaw_address')
pitch_channel          = params_.get('pitch_channel')
roll_channel           = params_.get('roll_channel')
offboard_channel       = params_.get('offboard_channel')

class DriveControllerHardware(DriveControllerBase):
    def __init__(self): 
        super().__init__()

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # initialize drive speed and turn speed
        self.drive_speed_in = dead
        self.turn_speed_in  = dead

        # Manual vs automatic control
        self.manual = True

        # roboclaw stuff
        self.address = 0x80
        self.rc = Roboclaw(drive_roboclaw_address,115200)
        self.rc.Open()

    def rc_listener_callback(self, msg):
        self.drive_speed_in  = msg.values[pitch_channel]
        self.turn_speed_in   = msg.values[roll_channel]

        # set manual or automatic control of tilt angle
        if msg.values[offboard_channel] == max:
            self.manual = False
        else:
            self.manual = True

    def normalize(self,drive_speed_in):
        return (drive_speed_in-dead)/(max-dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def on_shutdown(self):
        self.rc._port.close()
        self.get_logger().info("port closed !")

    def move_right_wheel(self, speed):
        if speed > 0:
            if speed >= 127:
                speed = 126
            self.rc.BackwardM1(self.address, abs(speed))
        else:
            if speed <= -127:
                speed = -126
            self.rc.ForwardM1(self.address, abs(speed))

    def move_left_wheel(self, speed):
        if speed > 0:
            if speed >= 127:
                speed = 126
            self.rc.ForwardM2(self.address, abs(speed))
        else:
            if speed <= -127:
                speed = -126
            self.rc.BackwardM2(self.address, abs(speed))

    def update(self):
        print("self.manual: ", self.manual)
        if (self.manual):
            # manual control of driving
            lin_vel = self.map_speed(self.normalize(self.drive_speed_in))
            ang_vel = self.map_speed(self.normalize(self.turn_speed_in))

            self.get_logger().info(f"lin_vel, ang_vel: ({lin_vel},{ang_vel})")

            self.move_right_wheel(lin_vel + ang_vel)
            self.move_left_wheel(lin_vel - ang_vel)
        else:
            # automatic control of driving
            lin_vel = -self.map_speed(self.drive_speed)
            ang_vel = -self.map_speed(self.turn_speed)
            self.move_right_wheel(lin_vel + ang_vel)
            self.move_left_wheel(lin_vel - ang_vel)
     
def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveControllerHardware()
    drive_controller.get_logger().info("Starting DriveControllerHardware node...")
    rclpy.spin(drive_controller)
    drive_controller.on_shutdown()  # do any custom cleanup
    drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()