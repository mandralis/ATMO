# ROS imports
import rclpy

# Numpy imports 
from numpy import clip,pi,rad2deg

# Morphing Lander imports
from atmo.mpc.TiltControllerBase import TiltControllerBase
from atmo.mpc.parameters import params_

v_max_absolute = params_['v_max_absolute']

class TiltSim(TiltControllerBase): 
    def __init__(self):
        super().__init__()
 
    def reset_encoder_trigger(self):
        pass

    def get_current_tilt_angle(self):
        print(f"tilt angle is: {rad2deg(self.tilt_angle)}")
        return self.tilt_angle

    def update(self):
        self.tilt_angle += float(self.Ts) * self.tilt_vel * v_max_absolute
        self.tilt_angle = clip(self.tilt_angle,0.0,pi/2)

def main(args=None):
    rclpy.init(args=args)
    print("Spinning TiltSim node \n")
    tilt_sim = TiltSim()
    rclpy.spin(tilt_sim)
    tilt_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
