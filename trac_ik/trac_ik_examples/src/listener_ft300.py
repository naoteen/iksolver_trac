#!/usr/bin/env python
import rospy
import sys
import numpy as np
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench

class RobotiqFTsensor(object):
    def __init__(self):
        rospy.Subscriber('robotiq_ft_wrench', WrenchStamped, self.callback)
        self.wrench = Wrench()
        self.wrench_smooth = Wrench()
        num = 30
        self.temp_force_x = np.zeros(num)
        self.temp_force_y = np.zeros(num)
        self.temp_force_z = np.zeros(num)
        self.temp_torque_x = np.zeros(num)
        self.temp_torque_y = np.zeros(num)
        self.temp_torque_z = np.zeros(num)
        
    def callback(self, data):
        self.wrench = data.wrench
        f_x = np.append(self.temp_force_x, self.wrench.force.x)
        f_y = np.append(self.temp_force_y, self.wrench.force.y)
        f_z = np.append(self.temp_force_z, self.wrench.force.z)
        t_x = np.append(self.temp_torque_x, self.wrench.torque.x)
        t_y = np.append(self.temp_torque_y, self.wrench.torque.y)
        t_z = np.append(self.temp_torque_z, self.wrench.torque.z)
        self.wrench_smooth.force.x = np.mean(f_x)
        self.wrench_smooth.force.y = np.mean(f_y)
        self.wrench_smooth.force.z = np.mean(f_z)
        self.wrench_smooth.torque.x = np.mean(t_x)
        self.wrench_smooth.torque.y = np.mean(t_y)
        self.wrench_smooth.torque.z = np.mean(t_z)
        self.temp_force_x = np.delete(f_x, 0)
        self.temp_force_y = np.delete(f_y, 0)
        self.temp_force_z = np.delete(f_z, 0)
        self.temp_torque_x = np.delete(t_x, 0)
        self.temp_torque_y = np.delete(t_y, 0)
        self.temp_torque_z = np.delete(t_z, 0)
        

if __name__ == '__main__':
    rospy.init_node("ft300_listener", disable_signals=True)
    ft300 = RobotiqFTsensor()

    while not rospy.is_shutdown():
        print ft300.wrench