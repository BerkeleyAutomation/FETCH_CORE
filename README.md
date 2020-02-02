# FETCH_CORE

This is our interface for the FETCH robot. Everything that doesn't need a display, including Moveit! can be run on the Fetch.

## Requirements (partial):

We develop locally on our FETCH Robot. All requirements are met locally. **You must setup your own virtual env**. If you need an example, check out `/d/`, `fetch_core` is a virtualenv that works and can be copied into your virtualenv flder. that works. To activate it, use `source ~/virtualenvs/fetch_core/bin/activate`


- Python 2.7.15
- Ubuntu 18.04
- ROS Melodic

## Quick Start

### Current Tutorial
1. Begin by following the [official installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). To validate your installation, run the [validation scripts from the FETCH repo](https://docs.fetchrobotics.com/indigo_to_melodic.html#post-install-validation). Make sure you go over the [robot hardware](https://docs.fetchrobotics.com/robot_hardware.html#mechanism-terminology) and [API][1] before starting to use the robot.
2.  Try to teleoperate your robot! First run `export ROS_MASTER_URI=http://<robot_name_or_ip>:11311` and then `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`.
3. Once your environment is setup. It should as simple as finding the script you want to run under `examples/` and executing `python SCRIPT-NAME.py`. If the script uses Moveit! as a library for motion planning, run `roslaunch fetch_moveit_config move_group.launch` in a seperate terminal window or tty.

Next Steps: Checkout the `examples` and read the `examples/README.md`! If you are interested in motion_planning check out that folder. All the files should run on the FETCH as just a regular python script given you have installed everything correctly.

### Daniel - Depreciated Tutorial

- [(DEPRECIATED) Daniel, Seimens Challenge] Use the skeleton code provided in `fetch_core/skeleton.py`. This provides minimum functionality to access all relevant parts of the code. For the Siemens challenge ([repository here][1]), we used a different skeleton class, `fetch_core/robot_interface.py`, which has some hard-coded base and head tilting for the challenge, but don't use that.
>>>>>>> ca49a29... Refactoring of codebase. There are tests to demo different aspecs of the fetch under basic_tests/ as well as metrics from my tests. There is a simplified example of a script that is used to run a linear joint trajectory. Also updated the README inside of examples with information on what is in each folder.

- Start off by using code similar to `examples/test_skeleton.py` which will give
  you an overview of common code usage. The `examples/` directory has other
  example code usage and screenshots of the Fetch in action.

TODOs:

- [ ] Develop simple test cases where we specify camera coordinates for a robot's gripper to go to, ideally by simply drawing a bounding box around a set of image pixels.  Then this will test if camera-to-workspace is succeeding. There isn't a clean test case for this, and the closest is based on the Siemens challenge code in tandem with `fetch_core/robot_interface.py` and `fetch_core/gripper.py`.

- [ ] It is not possible (or at least, *very* difficult) to do planning with the arms *and* the base simultaneously with the Fetch using MoveIt. Thus, we can't do the base planning like the HSR does, which means the quickest solution is to develop a heuristic: if the robot's Inverse Kinematics (IK) do not find a solution for the arm, then we can run IK by assuming that the base is located at a different spot (e.g., 0.01 meters forward), and keep trying different locations.

See `examples/` directory for starter code that we can use for these above points.

## SSH access

- You can ssh into the fetch from any machine with the default credentials at 
`fetch@fetch59.local`, password `robotics`. Create your own account and develop 
in your own home directory (give yourself sudo if necessary). The fetch is only
on the LAN. Make sure that when using it you have enough space.

- [ ] It is not possible (or at least, *very* difficult) to do planning with the arms *and* the base simultaneously with the Fetch using MoveIt. Thus, we can't do the base planning like the HSR does, which means the quickest solution is to develop a heuristic: if the robot's Inverse Kinematics (IK) do not find a solution for the arm, then we can run IK by assuming that the base is located at a different spot (e.g., 0.01 meters forward), and keep trying different locations.

See `examples/` directory for starter code that we can use for these above points.

## SSH access

- You can ssh into the fetch from any machine with the default credentials at 
`fetch@fetch59.local`, password `robotics`. Create your own account and develop 
in your own home directory (give yourself sudo if necessary). The fetch is only
on the LAN. Make sure that when using it you have enough space.


## Troubleshooting

- Make sure you are on the *Automation* WiFi with `ROS_MASTER_URI` set up
  appropriately (e.g., in `~/.bashrc`).

- Don't set the Fetch to be as high as 0.4m. The max height we should use in
  practice is probably around 0.35. Make sure the robot is not turned off at the
  max height, otherwise it may be necessary to physically press the robot down.
  (We had to talk with Fetch support about this.)

- If you get some missing trajectory service (e.g., of the Head) then try
  turning off the robot and the breaker, and then turning on the breaker and the
  ON button. Actually that's a good strategy in case anything looks odd.

- If the Joystick (for teleoperation) won't connect to the Fetch, use the USB
  cable to connect it with the Fetch head.

- When the robot is turned ON, the area where it is turned ON is considered the
  `odom` link. The `base_link` is also set at that position, but it moves in
  accordance with the bottom of the Fetch's base when it moves, whereas `odom`
  stays fixed. Thus, `odom` is like the `map` frame from the HSR.

- For the physical robot, for BOTH MoveIt! and rviz you need to export the
  correct `ROS_MASTER_URI` variable.

- Running moveit! might throw an error that is outside of 0.01 tolerance. You can run roslaunch for moveit and see the error details. It shouldn't matter. As of our version, there should be some way to publish 0.0 (so no tolerance check) or up the tolerance level, but we haven't done it since it doesn't matter for our tasks. Interestingly enough, it seems to self-correct after a while as well at least for `simple_disco.py` and `simple_wave.py` under examples.

For more information, see [the Fetch Docs][2].

## Tests

Look at `examples/`.

If you are using the physical Fetch robot, turn it on, and be sure your machine
is connected to the robot by being on the AUTOLAB WiFi and with `ROS_MASTER_URI`
set up appropriately (we have Fetch 59):

```
export ROS_MASTER_URI=http://fetch59.local:11311
```

Alternatively, have the Gazebo simulator running if you just want to run
simulation:

```
roslaunch fetch_gazebo simulation.launch
```

with `ROS_MASTER_URI=localhost:11311`. In either case, any time MoveIt is used
(for planning, computing inverse kinematics, etc.), you must have the following
command:

```
roslaunch fetch_moveit_config move_group.launch
```

running in the background, before calling whatever code you're using.

Once the setup is ready, just call `python [script_name]`.


[1]:https://github.com/BerkeleyAutomation/siemens_challenge
[2]:http://docs.fetchrobotics.com/index.html
[3]:http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
[4]:https://docs.fetchrobotics.com/api_overview.html
[5]:https://docs.ros.org/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html
