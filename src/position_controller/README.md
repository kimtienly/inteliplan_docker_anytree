# Position Controller

**A Position/Pose Controller for ANYmal**

This package is a fork from ori-drs/navigation_drs.

## Main Features

The basic behaviour of the controller is to take a goal pose (in x,y and yaw) and to walk to that pose. The
robot will:
1. Turn to face the goal position. 
2. Walk towards the goal position.
3. Turn at the goal position to the goal yaw.

The controller has been tested outdoors in a variety of conditions. Externals disturbances such as a wind,
terrain slope or miss calibration of the robot itself, cause the robot to drift in position. Simply commanding
zero position velocity when turning in place is not sufficient for the robot to hold position.

These modifications have been added:
1. In 'turn to face goal' mode, the robot will track to hold the starting position.
2. In 'walk to goal' mode, the robot will track towards the line between the start and the goal positions.
3. In 'turn to goal yaw' mode, ditto.

These tunings allow the robot to turn crisply, walk in a straightline and turn crisply at the end.
The state machine moves continuously from 1, 2 and then 3.

## Additional Features

There are some additional behaviours:

1. The main behaviour is to "turn-walk-turn" where the robot "turnsaround" to face the goal (in mode 1)
2. If a goal is behind the robot, instead the robot will turn to face away from the goal and walk  backwards to the goal. This mode is called "backwards". Its useful inside in the lab.
3. The final approach is called "shuffle". Basically the robot turns to face the goal while walking most directly to the goal. Basically, this is just mode 3 mentioned above.

The input is a PoseStamped message, in either odom or map frames. Tracking is carried out in odom frame.

There are a variety of interfaces supported:

* Dynamic Reconfigure: to change the desired velocity and walking modes.
* Topics interface to get a goal as well as cancel it.
* ActionLib Server interface: to allow programmatic control.
* A topic which accepts a PoseArray e.g. to go to a series of poses in order.