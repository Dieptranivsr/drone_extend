# drone_extend
`drone_extend` is an extension simulation of [px4_fast_planner](https://github.com/mzahana/px4_fast_planner). This repository is a version of `px4_fast_planner` with `D435i Depth Camera`.

<img src="https://user-images.githubusercontent.com/69444682/161436743-24bf3fba-152f-46b6-afeb-8c8111feed8b.png" width="425"> 

## Implement Outdoor
```
  $ roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"
  $ roslaunch realsense2_camera rs_camera.launch enable_color:=false depth_width:=640 depth_height:=480 depth_fps:=15
  $ rostopic echo /mavros/local_position/pose
  $ roslaunch px4_fast_planner outdoor.launch
  $ rosrun tf view_frames && evince frames.pdf
  $ rosbag record -o ~/ -a -x "(.*)theora(.*)|(.*)compressed(.*)"
  $ rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 19.0
    y: 15.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

## Simulate on Flightmare 

<p align="center">
  <img src="[files/raptor1.gif](https://user-images.githubusercontent.com/69444682/185907550-88583c10-ae46-41d1-bfcf-389d35678ad6.png)" width = "400"/>
  <img src="[files/raptor2.gif](https://user-images.githubusercontent.com/69444682/185909241-98553f82-a6cf-4d0c-baa2-c24c271b3147.gif)" width = "400"/>
  <img src="[files/icra20_2.gif](https://user-images.githubusercontent.com/69444682/185909614-d31190f0-477d-4995-b624-1b432260ce79.gif)" width = "400"/>
</p>

```
  $ roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"
  $ roslaunch realsense2_camera rs_camera.launch enable_color:=false depth_width:=640 depth_height:=480 depth_fps:=15
  $ rostopic echo /mavros/local_position/pose
  $ roslaunch px4_fast_planner outdoor.launch
  $ rosrun tf view_frames && evince frames.pdf
  $ rosbag record -o ~/ -a -x "(.*)theora(.*)|(.*)compressed(.*)"
  $ rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 19.0
    y: 15.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
