#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64MultiArray

def unified_cmd_pub():
    rospy.init_node('pub_two_cmd', anonymous=True)
    return rospy.Publisher('/unified_joint_cmd', Float64MultiArray, queue_size=10)

def init_cmd():
    cmdArr = Float64MultiArray()
    cmdArr.data.append(0.0)#j0
    cmdArr.data.append(0.0)#j1
    cmdArr.data.append(2.2)#j2
    cmdArr.data.append(2.9)#j3
    cmdArr.data.append(2.2)#j4
    cmdArr.data.append(0.0)#j5
    return cmdArr

def move_to(pub, cmdArr, joint, newcmd):
    cmdArr.data[joint] = newcmd
    pub.publish(cmdArr)



if __name__ == '__main__':
    pub = unified_cmd_pub()
    motor_cmd = init_cmd()

    rospy.sleep(1.)
    move_to(pub, motor_cmd, 0, 0.5)#0.9)

    rospy.sleep(1.5)
    move_to(pub, motor_cmd, 0, 0.8)# 0.5)