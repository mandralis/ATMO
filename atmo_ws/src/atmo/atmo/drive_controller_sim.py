import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from atmo.mpc.DriveControllerBase import DriveControllerBase

class DriveControllerSim(DriveControllerBase):
    def __init__(self):
        super().__init__('DriveControllerSim')

    def update():
        pass

def main(args=None):
    rclpy.init(args=args)
    print("Spinning DriveSim node \n")
    drive_sim = DriveControllerSim()
    rclpy.spin(drive_sim)
    drive_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()