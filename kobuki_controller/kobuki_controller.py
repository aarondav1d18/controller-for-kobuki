"""
This is an example ROS2 node using the UGRDV Kobuki ROS2 API. Its purpose is to demonstrate the full-stack functionality of the platform, including:
- Receiving camera images
- Processing camera images on the GPU
- Sending drive commands

It will follow the cone nearest to the centre of the camera frame.

One important thing to note is, if you do type annotations (you should try to!), avoid using `list[int] or tuple[int, int]` type annotations
 as the Koubki runs python3.6 which will crash when trying to parse these type annotations. 
You can still use type hints such as the following:
 - `def function(...) -> float`
 - etc...

"""
import rclpy
from rclpy.node import Node

import numpy as np

from ugrdv_kobuki_msgs.msg import DriveCommand
from ugrdv_kobuki_msgs.srv import EnableDrive
from sensor_msgs.msg import Joy


class ExampleKobukiNode(Node):
    def __init__(self) -> None:
        """
        The initialisation function of the node, this does a bunch of stuff and it should all be commented but it roughly boils down to:
        1. Instantiate subscriptions and publishers required for the function of the node
        2. Enable the Kobuki drive-train 
        """

        super().__init__('kobuki_controller')

        # Create subscriptions to the left camera frame topic
        self.drive_command_publisher = self.create_publisher(DriveCommand, '/ugrdv_kobuki/drive_command', 1)
        self.controller_inputs = self.create_subscription(Joy, '/joy', self.controller_state_changed, 1)

        # Enable the drivetrain - this will tell the ugrdv_kobuki_ros package that we want to be able to move the car
        client = self.create_client(EnableDrive, "ugrdv_kobuki/enable_drive")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Cannot find drive-train service, waiting')
        request = EnableDrive.Request()
        request.enable = True

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().warn('Drive-train service call failed')

    def convert_trigger_to_percentage(self,input) -> float:
        percentage = (50 * (1-input))/100
        return percentage

    def controller_state_changed(self,msg:Joy):
        '''
        max angular velocity = pi
        max linearvel = 1.5
        right left = -1, 1
        up and down = 1, -1
        trigger down = -1
        '''
        axis = msg.axes
        trigger_input = float(axis[5])
        left_stick_x = float(axis[0])
        linear_vel = 1.5 * self.convert_trigger_to_percentage(trigger_input)
        angular_vel = np.pi * left_stick_x
        drive = DriveCommand()
        drive.linearvel = linear_vel
        drive.angularvel = angular_vel

def main(args=None) -> None:
    """
    The entrypoint of the program, this will create the ROS2 node, tell the ROS2 runtime to 'spin' it
    and then shutdown gracefully when the node quits
    """
    rclpy.init(args=args)
    kobuki_example_node = ExampleKobukiNode()
    rclpy.spin(kobuki_example_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
