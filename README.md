# FETCH_CORE

## Overview

This is our interface for the Fetch robot.

Troubleshooting:

- Make sure you are on the AUTOLAB WiFi with `ROS_MASTER_URI` set up
  appropriately.

- Don't set the Fetch to be as high as 0.4m. The max height we should use in
  practice is probably around 0.35. Make sure the robot is not turned off at the
  max height, otherwise it may be necessary to physically press the robot down.

- If you get some missing trajectory service (e.g., of the Head) then try
  turning off the robot and the breaker, and then turning on the breaker and the
  ON button.

## Tests

To test these, make sure the physical Fetch is on (or the Gazebo simulator is
running with some launch file) and that your machine is connected to the robot
by being on the AUTOLAB WiFi and with `ROS_MASTER_URI` set up appropriately.

Any time MoveIt is used (for planning, computing inverse kinematics, etc.), you
must have the following command:

```
roslaunch fetch_moveit_config move_group.launch
```

running in the background, for both the simulator and the real robot, before
calling these scripts.

Once the setup is ready, just call `python [script_name]`.

- `test_robot_interface.py` for basic tests of gripper, cameras, head movement,
  torso.

  **Status**: passing for both simulator and physical robot

- `test_base_and_position.py` for testing if we can move back to a specified
  starting location, also testing base movement more generally

  **Status**: passing for simulator only

- `test_move_to_pose.py` for moving the end-effectors to certain locations.

  **Status**: have not tested
