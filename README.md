Controls OpenManipulator-X with a Xbox 360 gamepad. Forked from https://github.com/ros-teleop/teleop_twist_joy and used it as an template.

Subscribes to:

 - joy, gamepad analog axis and button presses
 - joint_states, joint positions in radians published by opencr core

Publishes to:

 - joint_trajectory_point, openmanipulator control in joint space
 - joint_move_time, openmanipulator trajectory speed
 - gripper_position, openmanipulator gripper position
 - gripper_move_time, openmanipulator gripper speed
