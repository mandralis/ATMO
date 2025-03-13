import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry  # Import the custom message

class MocapToVisualOdometry(Node):

    def __init__(self):
        super().__init__('mocap_to_visual_odometry')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/m4_base/pose',
            self.mocap_pose_callback,
            10)
        self.publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10)

    def mocap_pose_callback(self, msg): 
        # Process the PoseStamped message and publish to /fmu/in/vehicle_visual_odometry

        vehicle_odom_msg = VehicleOdometry()

        # Set the timestamp
        # vehicle_odom_msg.timestamp = int((msg.header.stamp.sec * 1e6) + (msg.header.stamp.nanosec / 1e3))
        vehicle_odom_msg.timestamp = 0
        vehicle_odom_msg.timestamp_sample = 0

        # Set the pose_frame (assuming NED for simplicity)
        vehicle_odom_msg.pose_frame = 2

        # Set position
        vehicle_odom_msg.position = [msg.pose.position.x, msg.pose.position.z, -msg.pose.position.y]

        # Set orientation (using quaternion)
        vehicle_odom_msg.q = [msg.pose.orientation.w, -msg.pose.orientation.z, msg.pose.orientation.x, -msg.pose.orientation.y]

        # Set velocity_frame (assuming NED for simplicity)
        vehicle_odom_msg.velocity_frame = 2
        vehicle_odom_msg.velocity = [0.0, 0.0, 0.0] # in m/s
        vehicle_odom_msg.angular_velocity = [0.0, 0.0, 0.0] # in body-fixed frame (rad/s)
        vehicle_odom_msg.position_variance = [0.0, 0.0, 0.0]
        vehicle_odom_msg.orientation_variance = [0.0, 0.0, 0.0]
        vehicle_odom_msg.velocity_variance = [0.0, 0.0, 0.0]

        # Set velocity, angular_velocity, and other fields as needed

        # Publish the transformed message
        self.publisher.publish(vehicle_odom_msg)

def main(args=None):
    rclpy.init(args=args)

    mocap_to_visual_odometry_node = MocapToVisualOdometry()

    rclpy.spin(mocap_to_visual_odometry_node)

    mocap_to_visual_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
