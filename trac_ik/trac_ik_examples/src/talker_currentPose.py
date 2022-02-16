#!/usr/bin/env python3
import moveit_commander
import rospy
import math
import sys    
from geometry_msgs.msg import PoseStamped

"""
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.11.6
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=false
rosbag record /ur_current_cartesian_pose
"""
if __name__ == '__main__':
    robot = moveit_commander.RobotCommander()
    mani = moveit_commander.MoveGroupCommander("manipulator")
    rospy.init_node('current_pose', anonymous=True)
    pub = rospy.Publisher('ur_current_cartesian_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        pose = mani.get_current_pose()
        pub.publish(pose)
        print(pose)
        rate.sleep()