# kobuki_controller

This project was made using custom msg types for ros2 that drive the robot.

## What it does
This will take a controller input using the joy node that is implemented within ros2 using a subscriber. From this it will convert the input that is given from this to a linear velocity and a angular velocity. These are capped at the top of the `kobuki_controller.py` file and can be changed.
