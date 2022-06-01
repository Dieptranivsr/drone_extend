Note:
> - [x] parameter
> - [x] feasibility: tends to lengthen the overall trajectory while pushing it far from obstacles
> - [ ] show cost_function value => type of algorithm Nplot  [Local gradient-based optimization](https://github.com/stevengj/nlopt/blob/master/doc/docs/NLopt_Algorithms.md#local-gradient-based-optimization)

* parameter
```xml
  <!-- main node -->
  <node pkg="plan_manage" name="fast_planner_node" type="fast_planner_node" output="screen">
    <!-- replanning method -->
    <param name="planner_node/planner" value="1" type="int"/> <!-- kino_plan = 1; topo_plan = 2 -->
    ...
    <!-- planning fsm -->
    <param name="fsm/flight_type" value="$(arg flight_type)" type="int"/> <!-- target type - click_point in rviz / waypoints -->
    <param name="fsm/thresh_replan" value="1.5" type="double"/> <!-- check end_pos near end -->
    <param name="fsm/thresh_no_replan" value="2.0" type="double"/> <!-- check start_pos near start -->

    <param name="sdf_map/local_update_range_x"  value="5.5" /> 
    <param name="sdf_map/local_update_range_y"  value="5.5" /> <!-- local update range of sdf map -->
    <param name="sdf_map/local_update_range_z"  value="4.5" /> 
    <param name="sdf_map/obstacles_inflation"     value="0.099" /> 
    <param name="sdf_map/local_bound_inflate"    value="0.0"/>
    <param name="sdf_map/local_map_margin" value="50"/>
    <param name="sdf_map/ground_height"        value="-1.0"/> <!-- ground height -->
    ...
    <!-- local fusion -->
    <param name="sdf_map/p_hit"  value="0.65"/>
    <param name="sdf_map/p_miss" value="0.35"/>
    <param name="sdf_map/p_min"  value="0.12"/>  <!-- probability hit and miss -->
    <param name="sdf_map/p_max"  value="0.90"/>
    <param name="sdf_map/p_occ"  value="0.80"/>
    <param name="sdf_map/min_ray_length" value="0.5"/>
    <param name="sdf_map/max_ray_length" value="4.5"/>

    <param name="sdf_map/esdf_slice_height" value="0.3"/> <!-- esdf slice height -->
    <param name="sdf_map/visualization_truncate_height"   value="2.49"/> <!-- visualization truncate height -->
    <param name="sdf_map/virtual_ceil_height"   value="2.5"/> <!-- virtual ceil height -->
    ...

  <!-- planner manager -->
    ...
    <param name="manager/control_points_distance" value="0.5" type="double"/> <!-- control points distance -->
    ...
    <param name="manager/use_kinodynamic_path" value="true" type="bool"/>
    <param name="manager/use_optimization" value="true" type="bool"/>

  <!-- kinodynamic path searching -->
    <param name="search/max_tau" value="0.6" type="double"/>  <!-- search in open_set -->
    <param name="search/init_max_tau" value="0.8" type="double"/>
    ...
    <param name="search/w_time" value="10.0" type="double"/> <!-- d coefficient in J function -->
    <param name="search/horizon" value="7.0" type="double"/> <!-- reach_horizon range -->
    <param name="search/lambda_heu" value="5.0" type="double"/>
    <param name="search/resolution_astar" value="0.1" type="double"/> <!-- respectively with resolution of map -->
    <param name="search/time_resolution" value="0.8" type="double"/>
    <param name="search/margin" value="0.2" type="double"/>
    <param name="search/allocate_num" value="100000" type="int"/>
    <param name="search/check_num" value="5" type="int"/> <!-- check safety from cur_state (respectively with voxel unit) -->

  <!-- trajectory optimization --> <!-- lambda coefficient for gradient -->
    <param name="optimization/lambda1" value="10.0" type="double"/> <!-- SMOOTHNESS -->
    <param name="optimization/lambda2" value="5.0" type="double"/> <!-- DISTANCE -->
    <param name="optimization/lambda3" value="0.00001" type="double"/> <!-- FEASIBILITY -->
    <param name="optimization/lambda4" value="0.01" type="double"/> <!-- ENDPOINT -->
    <param name="optimization/lambda7" value="100.0" type="double"/> <!-- WAYPOINTS -->
    <param name="optimization/dist0" value="0.4" type="double"/> <!-- BsplineOptimizer::calcDistanceCost(cost, gradient) -->
    ...

    <param name="optimization/algorithm1" value="15" type="int"/> <!-- use while doing optimization using NLopt slover -->
    <param name="optimization/algorithm2" value="11" type="int"/>

    <param name="optimization/max_iteration_num1" value="2" type="int"/>
    <param name="optimization/max_iteration_num2" value="300" type="int"/> <!-- only use this param -->
    <param name="optimization/max_iteration_num3" value="200" type="int"/>
    <param name="optimization/max_iteration_num4" value="200" type="int"/>

    <param name="optimization/max_iteration_time1" value="0.0001" type="double"/>
    <param name="optimization/max_iteration_time2" value="0.005" type="double"/> <!-- only use this param -->
    <param name="optimization/max_iteration_time3" value="0.003" type="double"/>
    <param name="optimization/max_iteration_time4" value="0.003" type="double"/>

    <param name="optimization/order" value="3" type="int"/>

    <param name="bspline/limit_vel" value="$(arg max_vel)" type="double"/>
    <param name="bspline/limit_acc" value="$(arg max_acc)" type="double"/>
    <param name="bspline/limit_ratio" value="1.1" type="double"/>

    <param name="bspline/limit_vel" value="$(arg max_vel)" type="double"/>
    <param name="bspline/limit_acc" value="$(arg max_acc)" type="double"/>
    <param name="bspline/limit_ratio" value="1.1" type="double"/>
```

```
Although we constrain kinodynamic feasibility in the path searching and 
optimization, sometimes we get infeasible trajectories. The basic reason 
is that gradient information tends to lengthen the overall trajectory 
while pushing it far from obstacles. Consequently, the quadrotor has to 
fly more aggressively in order to travel longer distance within the same 
time, which unavoidably causes over aggressive motion if the original 
motion is already near to the physical limits.
```
