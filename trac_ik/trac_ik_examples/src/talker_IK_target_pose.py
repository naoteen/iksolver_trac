#!/usr/bin/env python3
import rospy
import sys
import math
import moveit_commander
from geometry_msgs.msg import PoseStamped

'''
Control real robot
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.11.3
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
roslaunch trac_ik_examples tracik.launch

Gazebo simulation
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch trac_ik_examples tracik.launch sim:=true
'''

class targetTalker(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.mani = moveit_commander.MoveGroupCommander("manipulator")
        self.pub = rospy.Publisher('IK_target_pose', PoseStamped, queue_size=10)
        rospy.sleep(1)

    # trackIK
    def setPose(self, pose):
        if self.checkSafety(pose):
            ps = PoseStamped()
            ps.pose = pose
            self.pub.publish(ps)
            self.pre_pose = pose

    # # moveit
    # def setPose(self, pose):
    #     if self.checkSafety(pose):
    #         self.mani.set_pose_target(pose)
    #         _, plan, _, _ = self.mani.plan()
    #         self.mani.execute(plan)

    def setJointAngles(self, angle):
        self.mani.clear_pose_targets()
        self.mani.set_joint_value_target(angle)
        self.mani.go()

    def initPose(self):
        angle_init  = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        self.setJointAngles(angle_init)

    def moveCartesianSpace(self, x, y, z):
        pose = self.mani.get_current_pose().pose
        pose.position.x += x
        pose.position.y += y
        pose.position.z += z
        self.setPose(pose)

    def checkSafety(self, pose):
        if pose.position.z > 0.2:
            return True
        else:
            return False


if __name__ == '__main__':
    # args = sys.argv
    rospy.init_node("IK_target_takler", disable_signals=True)
    target = targetTalker()
    target.initPose()
    
    scale = 1
    while not rospy.is_shutdown():
        x = 0.0
        y = 0.0
        z = 0.0
        key = input('command: ')
        if key == 'w':
            x = -0.01*scale
        if key == 's':
            x = 0.01*scale
        if key == 'a':
            y = -0.01*scale
        if key == 'd':
            y = 0.01*scale
        if key == 'q':
            z = 0.01*scale
        if key == 'e':
            z = -0.01*scale

        if key == 'state':
            print(target.mani.get_current_pose().pose)
        if key == 'scale':
            key = input('current is {}, change into ' .format(scale))
            scale = float(key)
        
        target.moveCartesianSpace(x,y,z)
