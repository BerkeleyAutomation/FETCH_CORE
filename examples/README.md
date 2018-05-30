# Test Examples

Examples to test out the Fetch. See repository README for status of which tests
are passing, and usage.



## Moving to Poses

**TODO TESTING IN PROGRESS**

This is the most important kind of test for us so please hurry up ...


## Base Rotation and Forward Movement

From running `python test_base_and_position.py`:

### Rotation

Testing the base *rotation* movement, where I make the robot turn X degrees,
then -X degrees, and try to see if it comes back to the target.

```
Results at -90 then 90 degrees, speed 0.3
(1) success, 1-2 distance, angle off 5 deg, went backwards
(2) failed first turn, only -20 deg
(3) failed first turn, only -10 deg
(4) failed first turn, only -5 deg
(5) failed second turn, but first angle overshot target by 20 deg

Results at 30 degrees, then -30 degrees, speed 0.3
(1) success, back 2cm, angle off 5 deg, went backwards
(2) success, back 2cm, angle good, went backwards
(3) failed first turn, only 5 deg
(4) failed first turn, only 2 deg
(5) success, back 2cm, angle good, went backwards

Results at -30 degrees, then 30 degrees, speed 0.3
(1) success, but after long delay (~20 sec) on _second_ turn
(2) success, but I had to physically nudge the robot since it was slow/stuck
(3) success, angle great back 2cm
(4) success, angle off 10 deg, back 2cm
(5) success, angle good, back 2cm
```

All result angles are obviously eyeball estimates. If it fails, it was
stuck for 10 or more seconds (I counted). After the first few trials it's
clear that the rotation axis isn't at the true center since the robot
repeatedly moves _backwards_ despite rotating X and then -X degrees. All
this is with the torso lowered to 0.03m to be safe.

### Forward Movement

Testing the base *forward* movement with `test_forward` method for moving the
robot 0.1 meters (10 centimeters) each time:

```
Now test with speed = 0.1 meters/sec

[WallTime: 1527719306.638319] start_pose: [-0.0096 -0.0001  0.9977]
[WallTime: 1527719309.881714] pose:       [ 0.1276  0.0007  0.3874]
[WallTime: 1527719309.882126] xy distance: 0.1372

[WallTime: 1527719367.568807] start_pose: [ 0.1276  0.0007  0.3874]
[WallTime: 1527719370.772539] pose:       [ 0.2615  0.0015  0.3729]
[WallTime: 1527719370.772919] xy distance: 0.1339

[WallTime: 1527719418.625483] start_pose: [ 0.2615  0.0015  0.3729]
[WallTime: 1527719421.869432] pose:       [ 0.3982  0.0024  0.3729]
[WallTime: 1527719421.870001] xy distance: 0.1368

[WallTime: 1527719458.508835] start_pose: [ 0.3982  0.0024  0.3729]
[WallTime: 1527719461.710364] pose:       [ 0.5315  0.0032  0.3293]
[WallTime: 1527719461.710722] xy distance: 0.1333

[WallTime: 1527719500.833941] start_pose: [ 0.5315  0.0032  0.3293]
[WallTime: 1527719504.037288] pose:       [ 0.6664  0.0039  0.2131]
[WallTime: 1527719504.037762] xy distance: 0.1349

Now test with speed = 0.2 meters/sec

[WallTime: 1527719599.782686] start_pose: [ 0.6664  0.0039  0.2131]
[WallTime: 1527719602.625932] pose:       [ 0.8543  0.0043  0.0969]
[WallTime: 1527719602.626325] xy distance: 0.1880

[WallTime: 1527719645.532461] start_pose: [ 0.8543  0.0043  0.0969]
[WallTime: 1527719648.535771] pose:       [ 1.0376  0.0036 -0.2664]
[WallTime: 1527719648.536151] xy distance: 0.1833

[WallTime: 1527719693.510821] start_pose: [ 1.0376  0.0036 -0.2664]
[WallTime: 1527719696.314423] pose:       [ 1.2232  0.0022 -0.4553]
[WallTime: 1527719696.314802] xy distance: 0.1856

[WallTime: 1527719738.526260] start_pose: [ 1.2232  0.0022 -0.4553]
[WallTime: 1527719741.369464] pose:       [ 1.4047  0.0006 -0.5279]
[WallTime: 1527719741.369885] xy distance: 0.1816

[WallTime: 1527719788.585682] start_pose: [ 1.4047  0.0006 -0.5279]
[WallTime: 1527719791.388903] pose:       [ 1.597  -0.0015 -0.6441]
[WallTime: 1527719791.389317] xy distance: 0.1923
```

These results imply that commanding a 10cm move means the robot actually moves
about 13.5cm at speed 0.1 and 18.5cm at speed 0.2, so it badly overshoots.  As
expected, faster speeds overshoot. I think overshooting happens because the code
always checks and waits if we've reached a certain distance, so it won't stop
beforehand. If the robot moves larger distances, for fixed speed, the
overshooting error should hopefully stay the same which means relatively it is
closer to the target.

Also, I turned the robot on and didn't modify the position before proceeding
with experiments, and I note that the y-coordinate can be off by a few
millimeters (should be 0 throughout). But that's probably not a concern.

Let's move for longer distances, so 20cm:

```
Now test with speed = 0.1 meters/sec

[WallTime: 1527723056.326300] start_pose: [   0.5742    0.1444  177.5047]
[WallTime: 1527723061.090067] pose:       [   0.3403    0.157   175.6014]
[WallTime: 1527723061.090457] xy distance: 0.2342

[WallTime: 1527723136.295692] start_pose: [   0.3403    0.157   175.6014]
[WallTime: 1527723139.537986] pose:       [   0.102     0.1747  175.7031]
[WallTime: 1527723139.538535] xy distance: 0.2390

[WallTime: 1527723177.866473] start_pose: [   0.1031    0.1746  175.7031]
[WallTime: 1527723181.308761] pose:       [  -0.1296    0.1919  175.7612]
[WallTime: 1527723181.309201] xy distance: 0.2333

Now test with speed = 0.05 meters/sec

[WallTime: 1527723225.764847] start_pose: [  -0.1296    0.1919  175.7612]
[WallTime: 1527723231.407195] pose:       [  -0.3453    0.2078  175.7321]
[WallTime: 1527723231.407562] xy distance: 0.2163

[WallTime: 1527723269.286191] start_pose: [  -0.3443    0.2077  175.7176]
[WallTime: 1527723274.568423] pose:       [  -0.5602    0.2238  175.674 ]
[WallTime: 1527723274.568809] xy distance: 0.2165

[WallTime: 1527723446.533462] start_pose: [  -0.5602    0.2238  175.674 ]
[WallTime: 1527723451.775800] pose:       [  -0.7754    0.2401  175.6304]
[WallTime: 1527723451.776196] xy distance: 0.2158
```

With a speed of 0.1 m/sec we get the same errors as before, as expected. With
0.05 m/sec we halve the error. So it seems like 0.05 m/sec could be a reasonable
speed for us.

Of course this assumes that the distance returned from the odometry base is
actually accurate (that's the thing that lets us detect `start_pose` and
`pose`).
