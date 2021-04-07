# SRC2-Sensor Fusion
Need: SRC2-vo-ros (+SRC2-image-selector), SRC2-localization-wo, SRC2-driving(teleop), SRC2-gazebo-truth-odom
Running scout_sensor_fusion.launch 
(running scout_sf_combined.launch will launch all of these)
```
roslaunch wvu_vo_ros vo_pipelineScout.launch
roslaunch rover_deadreckoning scout_dead_reckoning.launch
roslaunch teleop_modes scout_teleop_modes.launch
roslaunch gazebo_truth_odom scout_gazebo_truth_odom.launch
```

To launch the EKF that fuses gyro rates and compass measurements (noisy), to obtain a smoothed attitude estimation:
```
roslaunch sensor_fusion scout_sensor_fusion.launch
```
Call true_pose service
```
rosservice call /small_scout_1/true_pose true
```
