#!/bin/bash
# camera,map,range,path,traj,cmd

rosbag record /camera/pose /camera/color/image_raw /camera/depth/image_raw /sdf_map/occupancy_inflate /sdf_map/update_range /planning_vis/trajectory /planning/position_cmd_vis /mavros/local_position/pose 
