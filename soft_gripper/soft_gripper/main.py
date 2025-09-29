import rclpy
import yaml
import os

from ament_index_python.packages import get_package_share_directory
from soft_gripper.gripper_ros2_node import GripperROS2Node
from soft_gripper.waveshare_st_client import WaveshareStClient
from soft_gripper.gripper_controller import GripperController

def main(args=None):
    rclpy.init(args=args)

    # Get package share directory
    package_share_dir = get_package_share_directory('soft_gripper')

    # Load gripper configuration from install space
    config_path = os.path.join(package_share_dir, 'config', 'gripper_config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    motor_ids = config['motor_ids']
    port = config.get('port', '/dev/ttyACM0')
    baudrate = config.get('baudrate', 1000000)

    # Initialize Dynamixel client and gripper controller
    st_client = WaveshareStClient(motor_ids, port=port, baudrate=baudrate)
    gripper = GripperController(st_client, motor_ids, config)
    gripper.connect()

    # Start ROS 2 node
    gripper_node = GripperROS2Node(gripper)

    try:
        rclpy.spin(gripper_node)
    except KeyboardInterrupt:
        gripper_node.get_logger().info('Shutting down.')

    # Cleanup
    gripper_node.destroy_node()
    gripper.disconnect()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
