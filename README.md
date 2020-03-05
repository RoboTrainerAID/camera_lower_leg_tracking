camera_lower_leg_tracking
==========================================

## ROS Distro Support

|         | Kinetic | Melodic |
|:-------:|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://gitlab.ipr.kit.edu/IIROB/camera_lower_leg_tracking/tree/kinetic-devel) | 
| Status  | [![build status](https://gitlab.ipr.kit.edu/IIROB/camera_lower_leg_tracking/badges/kinetic-devel/pipeline.svg)](https://gitlab.ipr.kit.edu/$NAMESPACE$/camera_lower_leg_tracking/commits/kinetic-devel) | |


There are 3 nodes.
feet_detection_node is the whole project and should be started with the feet_detection.launch file.
toe_detection_node only detects the toes and should be startet with the toe_detection.launch file.
The fix_foot_axis is a node with the only purpose to fix the import problem of an arrow visualization marker into matlab. It subscribes to the marker and publishes its direction into a PointStamped. This node should only be used as a hack to fix that bug.


How to start the Leg Detection?

First make shure the Camera publishes pointscloud in the "/camera/depth_registered/points" Topic

If the Parameter "perform_side_init" is set to -1 in the launch File the Side Init will be skipped and the paramters from the launch File will be used.
If "perform_side_init" is set to anything else the SideInit will be performed.
The next Step is the FrontalInit.

After both Inits the Algorithm will publish the Koordinates of the Points to the following topics
    /camera_lower_leg_tracking/left_toe
    /camera_lower_leg_tracking/right_toe
    /camera_lower_leg_tracking/left_ankle
    /camera_lower_leg_tracking/right_ankle
    /camera_lower_leg_tracking/left_heel
    /camera_lower_leg_tracking/right_heel
    
The Axis of the Lower Legs will be published in this topics
    /camera_lower_leg_tracking/left_foot_axis
    /camera_lower_leg_tracking/right_foot_axis
    
