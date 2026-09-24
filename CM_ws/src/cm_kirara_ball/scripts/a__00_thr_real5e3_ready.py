#!/usr/bin/python



from trajectory_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import numpy as np
import actionlib
import threading

UR5e_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']
UR3_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']



def ur3_pub():
    
    theta7 = 0.4 * np.pi
    theta8 = -0.5 * np.pi
    theta9 = -0.5 * np.pi
    theta10 = -0.5 * np.pi
    theta11 = 0
    theta12 = 0
    
    
    
    
    
    
    
    traj2 = JointTrajectory()
    traj2.header = Header()
    
    traj2.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                         'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                         'wrist_3_joint']
    pub2 = rospy.Publisher('right/scaled_pos_joint_traj_controller/command',
                           JointTrajectory,
                           queue_size=10)
    while not rospy.is_shutdown():
        traj2.header.stamp = rospy.Time.now()
        pts2 = JointTrajectoryPoint()
        pts2.positions = [theta7, theta8, theta9, theta10, theta11, theta12]
        pts2.velocities = [0] * 6
        pts2.time_from_start = rospy.Duration(5.0)
        traj2.points = []
        traj2.points.append(pts2)
        pub2.publish(traj2)


def ur5e_pub():
    
    theta1 = -0.4 * np.pi
    theta2 = -0.5 * np.pi
    theta3 = 0.5 * np.pi
    theta4 = -0.5 * np.pi
    theta5 = 0
    theta6 = 0
    
    
    
    
    
    
    
    traj = JointTrajectory()
    traj.header = Header()
    
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                         'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                         'wrist_3_joint']
    pub = rospy.Publisher('left/scaled_pos_joint_traj_controller/command',
                          JointTrajectory,
                          queue_size=10)
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        pts.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        pts.velocities = [0] * 6
        pts.time_from_start = rospy.Duration(5.0)
        traj.points = []
        traj.points.append(pts)
        pub.publish(traj)


def main():
    rospy.init_node('ur5e_ur3_hand_mode')
    
    left_thread = threading.Thread(target=ur5e_pub)
    right_thread = threading.Thread(target=ur3_pub)
    left_thread.start()
    right_thread.start()



    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    
    main()
    
    
