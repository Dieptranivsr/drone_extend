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
