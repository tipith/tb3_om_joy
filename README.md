# tb3_om_joy

Controls OpenManipulator-X with a Xbox 360 gamepad. Forked from https://github.com/ros-teleop/teleop_twist_joy and used it as an template.

Subscribes to:

 - joy, gamepad analog axis and button presses
 - joint_states, joint positions in radians published by opencr core

Publishes to:

 - joint_trajectory_point, openmanipulator control in joint space
 - joint_move_time, openmanipulator trajectory speed
 - gripper_position, openmanipulator gripper position
 - gripper_move_time, openmanipulator gripper speed

# launch

To work with the tutorials at http://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation ROS_NAMESPACE is required. Also see list of required dependencies in the link. Launch with the following commands:

 - sudo xboxdrv --silent
 - ROS_NAMESPACE=om_with_tb3 roslaunch tb3_om_joy teleop.launch joy_dev:=/dev/input/jsX

Starts the following nodes:
 
 - joy_node, interfaces with xboxdrv and publishes /joy
 - teleop_twist_joy, publishes /cmd_vel
 - tb3_om_joy, this package, see above documentation