## Example with GUI
```
roslaunch quadrotor_simulator rviz.launch
roslaunch mav_manager demo.launch sim:=true vicon:=false
```

Run a simple example script
```
cd quadrotor_control/mav_manager/scripts
./sample.bash
```

There is also a GUI that can be used to send simple commands to the robot through the `mav_manager`. Launch it by running
```
rosrun rqt_mav_manager rqt_mav_manager
```
then try Motors ON -> Take Off -> Go To (set z > 0)

## Example with Waypoint Navigation Tool

Clone the [waypoint_navigation_tool](https://github.com/KumarRobotics/waypoint_navigation_plugin) in your workspace.

```
roslaunch quadrotor_simulator wp_nav.launch
roslaunch mav_manager demo.launch sim:=true vicon:=false
rosrun trackers_manager waypoints_to_action.py __ns:=quadrotor
```

Use rqt to start motors and takeoff.
```
rosrun rqt_mav_manager rqt_mav_manager
```
 * then try Motors ON -> Take Off -> Go To (set z > 0)

Use rviz to place waypoints and publish on the topic `/waypoints`. The `waypoints_to_action` node listens to this topic, sends an action goal to `TrajectoryTracker` and switches the tracker.