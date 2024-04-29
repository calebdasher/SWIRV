#!/usr/bin/env python3
# from section 3 of http://wiki.ros.org/roslaunch/API%20Usage
import roslaunch
import rospy
import atexit


rospy.init_node('Lidar', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jetson/Desktop/lidar_ws/src/unilidar_sdk/unitree_lidar_ros/src/unitree_lidar_ros/launch/run_without_rviz.launch"])
launch1.start()
rospy.loginfo("started unitree")

launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jetson/Desktop/lidar_ws/src/point_lio_unilidar/launch/mapping_unilidar.launch"])
launch2.start()
rospy.loginfo("started point lio")

try:
    launch1.spin()
    launch2.spin()
finally:
    launch1.shutdown()
    launch2.shutdown()
