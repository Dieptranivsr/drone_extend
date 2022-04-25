```xml
  <!-- kino_algorithm.xml -->
  <!-- planner manager -->
    <param name="manager/max_vel" value="$(arg max_vel)" type="double"/>
    <param name="manager/max_acc" value="$(arg max_acc)" type="double"/>
    <param name="manager/max_jerk" value="4" type="double"/>
    <param name="manager/dynamic_environment" value="0" type="int"/>
    <param name="manager/local_segment_length" value="6.0" type="double"/>
    <param name="manager/clearance_threshold" value="0.2" type="double"/>
    <param name="manager/control_points_distance" value="1" type="double"/>
```
```xml
  <!-- kino_replan.launch -->
  <!-- global parameters -->
  <arg name="max_vel" default="1" />
  <arg name="max_acc" default="2.5" />
```

```xml
  <!-- kino_replan.launch -->
    <!-- 1: use 2D Nav Goal to select goal  -->
    <!-- 2: use global waypoints below  -->
    <arg name="flight_type" value="2" />
    
    <!-- global waypoints -->
    <!-- If flight_type is set to 2, the drone will travel these waypoints one by one -->
    <arg name="point_num" value="2" />

    <arg name="point0_x" value="5.0" />
    <arg name="point0_y" value="0.0" />
    <arg name="point0_z" value="3.0" />

    <!-- set more waypoints if you need -->
    <arg name="point1_x" value="0.0" />
    <arg name="point1_y" value="0.0" />
    <arg name="point1_z" value="3.0" />

    <arg name="point2_x" value="0.0" />
    <arg name="point2_y" value="0.0" />
    <arg name="point2_z" value="3.0" />
```
