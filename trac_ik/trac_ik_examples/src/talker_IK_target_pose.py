#!/usr/bin/env python
import listener_ft300
import rospy
import tf
import sys
import math
import moveit_commander
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Wrench

'''
Control real robot
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.11.0
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
roslaunch trac_ik_examples tracik.launch

Gazebo simulation
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch trac_ik_examples tracik.launch sim:=true

rosrun trac_ik_examples talker_ft300.py 
'''

class targetTalker(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.mani = moveit_commander.MoveGroupCommander("manipulator")
        self.pub = rospy.Publisher('IK_target_pose', PoseStamped, queue_size=10)
        rospy.sleep(1)

    "choose the solver, trackIK or moveit"
    # trackIK
    # def setPose(self, pose):
    #     if self.checkSafety(pose):
    #         ps = PoseStamped()
    #         ps.pose = pose
    #         self.pub.publish(ps)
    #         self.pre_pose = pose

    # moveit
    def setPose(self, pose):
        if self.checkSafety(pose):
            self.mani.set_pose_target(pose)
            _, plan, _, _ = self.mani.plan()
            self.mani.execute(plan)

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

    def generate3pointsPath(self, start, wpose, goal): # only for moveit
        self.mani.clear_pose_targets()
        self.mani.set_pose_target(start)
        self.mani.go()
        waypoints = []
        waypoints.append(start)
        waypoints.append(wpose)
        waypoints.append(goal)
        (plan, fraction) = self.mani.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan

    def driveFreeForce(self, wrench):
        "Direction depends on the real FT300 pose. Please check."
        scale_f = -1e-4
        scale_t = -1e-2
        pose = self.mani.get_current_pose().pose
        p = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        wrench_q = tf.transformations.quaternion_from_euler(scale_t * wrench.torque.y, scale_t * wrench.torque.x, scale_t * wrench.torque.z)
        q = Quaternion(wrench_q[3], wrench_q[0], wrench_q[1], wrench_q[2]) 
        quat = q*p

        pose.position.x += scale_f * wrench.force.y
        pose.position.y += scale_f * wrench.force.x
        pose.position.z += scale_f * wrench.force.z

        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        pose.orientation.w = quat[0]

        print(q)
        self.setPose(pose)

    def checkSafety(self, pose):
        if pose.position.z > 0.2:
            return True
        else:
            return False


if __name__ == '__main__':
    # args = sys.argv
    rospy.init_node("IK_target_takler", disable_signals=True)
    rate = rospy.Rate(80)
    target = targetTalker()
    FT = listener_ft300.RobotiqFTsensor()

    target.initPose()
    
    scale = 1
    x = 0.0
    y = 0.0
    z = 0.0
    while not rospy.is_shutdown():
        key = raw_input('command: ')
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

    # while not rospy.is_shutdown():
    #     target.driveFreeForce(FT.wrench_smooth)
    #     rate.sleep()