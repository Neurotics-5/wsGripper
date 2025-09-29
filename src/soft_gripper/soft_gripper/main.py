import rclpy
import yaml, os
from ament_index_python.packages import get_package_share_directory
from soft_gripper.gripper_ros2_node import GripperROS2Node
from soft_gripper.waveshare_st_client import WaveshareStClient
from soft_gripper.gripper_controller import GripperController
from rclpy.executors import MultiThreadedExecutor   # <-- correct place

def main(args=None):
    rclpy.init(args=args)

    package_share_dir = get_package_share_directory('soft_gripper')
    config_path = os.path.join(package_share_dir, 'config', 'gripper_config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    motor_ids = config['motor_ids']
    port = config.get('port', '/dev/ttyACM0')
    baudrate = config.get('baudrate', 1000000)

    st_client = WaveshareStClient(motor_ids, port=port, baudrate=baudrate)
    gripper = GripperController(st_client, motor_ids, config)
    gripper.connect()

    node = GripperROS2Node(gripper)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()   # <-- no argument here
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        executor.shutdown()
        node.destroy_node()
        gripper.disconnect()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
