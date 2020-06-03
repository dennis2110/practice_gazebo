#!/usr/bin/env python
import rospy
import csv
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class Save_traj():
    def __init__(self):
        rospy.Subscriber("/realTTbot/motor_status", JointState, self.realTTbot_state_callback)
        rospy.Subscriber("/TTbot/joint_states", JointState, self.simTTbot_state_callback)
        rospy.Subscriber('/TTbot/save_joint_pos', Bool, self.save_pos_callback)
        
        self.is_save = False
        self.state = np.zeros((1,8))

    def simTTbot_state_callback(self, msg):
        self.state[0,0] = msg.position[0] #slide_pos
        self.state[0,1] = msg.position[1] #j1_pos
        self.state[0,2] = msg.position[2] #j2_pos
        self.state[0,3] = msg.position[3] #j3_pos
        
    def realTTbot_state_callback(self, msg):
        self.state[0,4] = msg.position[0] #slide_pos
        self.state[0,5] = msg.position[1] #j1_pos
        self.state[0,6] = msg.position[2] #j2_pos
        self.state[0,7] = msg.position[3] #j3_pos

    def save_pos_callback(self, msg):
        self.is_save = msg.data


if __name__ == '__main__':
    rospy.init_node('save_joint_pos',anonymous=True)
    save_traj = Save_traj()

    running_trajectory = []
    count = 0
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        if(save_traj.is_save):
            running_trajectory.append(save_traj.state[0].tolist())
            
        else:
            if(running_trajectory == []):
                pass
            else:
                with open('/home/lab606a/Documents/pingpong_trajectory/irl_trajectory/real_sim_traj_'+str(count+1)+'.csv', 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(running_trajectory)
                running_trajectory = []
                count += 1


        rate.sleep()

    

