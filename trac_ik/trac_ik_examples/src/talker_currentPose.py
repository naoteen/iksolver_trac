#!/usr/bin/env python
import talker_IK_target_pose
import rospy
import math
import sys    
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('current_pose', anonymous=True)
    pub = rospy.Publisher('ur_current_cartesian_pose', PoseStamped, queue_size=10)
    ur = talker_IK_target_pose.targetTalker()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        pose = ur.mani.get_current_pose()
        pub.publish(pose)
        print(pose)
        rate.sleep()