from trajectory_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import numpy as np
import actionlib
import threading
from geometry_msgs.msg import WrenchStamped
from robotiq_ft_sensor.msg import *

class KIRARA_LEFT_FT(object):
    def __init__(self):
        self.ft_data = np.array([0,0,0])
    def get_ft300_force(self):
        rospy.Subscriber("/left_ft300/robotiq_ft_sensor",ft_sensor,self.callback_ft,queue_size=10)
    def callback_ft(self,msg):
        self.ft_data = np.array([msg.Fx, msg.Fy, msg.Fz])
        
    def get_data(self):
        KIRARA_LEFT_FT.get_ft300_force(self)
        return self.ft_data

class KIRARA_RIGHT_FT(object):
    def __init__(self):
        self.ft_data = np.array([0,0,0])
    def get_ft300_force(self):
        rospy.Subscriber("/right_ft300/robotiq_ft_sensor",ft_sensor,self.callback_ft,queue_size=10)
    def callback_ft(self,msg):
        self.ft_data = np.array([msg.Fx, msg.Fy, msg.Fz])
        
    def get_data(self):
        KIRARA_RIGHT_FT.get_ft300_force(self)
        return self.ft_data

        
def main():
    
    rospy.init_node('ur5e_ur3_hand_mode')
    
    
    
    left_ft300 = KIRARA_LEFT_FT()
    right_ft300 = KIRARA_RIGHT_FT()
    left_ft300_init = np.array([0, 0, 0])
    right_ft300_init = np.array([0, 0, 0])
    while np.linalg.norm(left_ft300_init) == 0 or np.linalg.norm(right_ft300_init) ==0:
        left_ft300_init = left_ft300.get_data()
        right_ft300_init = right_ft300.get_data()
    print("left_ft_init = ",left_ft300_init)
    print("right_ft_init = ", right_ft300_init)



if __name__ == '__main__':
    
    main()