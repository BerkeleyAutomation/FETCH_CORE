# FETCH_CORE


## Overview

This is our interface for the Fetch robot. Please import `fetch_core` in your
code if using the Fetch, and go from there.

The `fetch_core/robot_interface.py` shows expected usage by importing a class
which contains the various aspects of interest (camera image, arm movement,
etc).

See the `examples/` directory for usage and pictures, and **later in this README
for the Siemens demonstration stuff**.


## Troubleshooting

- Make sure you are on the AUTOLAB WiFi with `ROS_MASTER_URI` set up
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
  `'odom'` link. So be careful if you set points with respect to that frame.

- For the physical robot, for BOTH MoveIt! and rviz you need to export the
  correct `ROS_MASTER_URI` variable.

## Tests

Look at `examples/`.

To test these, make sure the physical Fetch is on (or the Gazebo simulator is
running with some launch file) and that your machine is connected to the robot
by being on the AUTOLAB WiFi and with `ROS_MASTER_URI` set up appropriately.

Any time MoveIt is used (for planning, computing inverse kinematics, etc.), you
must have the following command:

```
roslaunch fetch_moveit_config move_group.launch
```

running in the background, for both the simulator and the real robot, before
calling these scripts. If running the simulator, then call something like this:

```
roslaunch fetch_gazebo simulation.launch
```

in a separate command line window.

Once the setup is ready, just call `python [script_name]`.

- `test_robot_interface.py` for basic tests of gripper, cameras, head movement,
  torso.

  **Status**: passing for both simulator and physical robot

- `test_base_and_position.py` for testing if we can move back to a specified
  starting location, also testing base movement more generally

  **Status**: passing for both simulator and physical robot, with some results
  recorded in `examples/README.md`.

- `test_move_to_pose.py` for moving the end-effectors to certain locations.

  **Status**: passing for the simulator, need to test more on the physical
  robot, but it's also working there. It helps to have the velocity scaling
  factor. Sometimes, though, there will be minor collisions between the arm and
  the base, which we should investigate in more detail. There is also an issue
  with wrist rotation.



## Siemens Demonstration

(TODO: notes in progress, need to integrate and clean up the siemens
demonstration repository.)

To run this with the Siemens demonstration, there are a few things to note.

The first step is to get a suitable world running. For this, we need to adjust
the toolboxes. Tools are organized in a directory like `screwdriver1`:

```
screwdriver1/
 241 Jun  8 15:28 model.config
1.5K Jun  8 15:56 model.sdf
317K Jun  8 15:55 screwdriver1.dae
```

There are three files: a configuration, an `.sdf`, and a `.dae`. The
configuration file needs to say version 1.4 for SDF (not 1.6). 

The `.sdf` also has to use version 1.4 (again, change from 1.6 to 1.4), *and*
it needs to have the correct URI pointing to the `.dae` file, like the one I
have here:

```
<uri>/home/daniel/FETCH_CORE/examples/screwdriver1/screwdriver1.dae</uri>
```

(Obviously the path will be different for your machine.) The `.dae` doesn't need
to be modified, but it needs to be put in the correct directories.

Then, the script that provides an API for spawning this needs to also point to
the correct directories for this ...

To test, start up a fresh simulator with the Fetch. Then run `python
spawn_object_script.py` and adjust the main method as desired.

However, you can't use `.obj` files for describing models; they need to be
`.dae`. If using the former we get:

```
Error [MeshManager.cc:88] Invalid mesh filename extension[/home/daniel/FETCH_CORE/examples/toolbox/tape3.obj] 
Error [Visual.cc:1668] Unable to create a mesh from /home/daniel/FETCH_CORE/examples/toolbox/tape3.obj
Error [Visual.cc:418] Ogre Error:OGRE EXCEPTION(2:InvalidParametersException): Header chunk didn't match either endian: Corrupted stream? in Serializer::determineEndianness at /build/buildd/ogre-1.8-1.8.1+dfsg/OgreMain/src/OgreSerializer.cpp (line 89)
```
