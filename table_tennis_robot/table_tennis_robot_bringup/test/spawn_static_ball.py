#!/usr/bin/env python
# license removed for brevity
import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

import csv




def spawm_ball(ballname, X, Y, Z):
    initial_pose = Pose()
    initial_pose.position.x = X
    initial_pose.position.y = Y
    initial_pose.position.z = Z
    f = open('/home/denn1slu/ros_workspace/vscode_ws/src/table_tennis_robot/pingpong_description/models/pingpong_static.sdf','r')
    sdff = f.read()
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(ballname, sdff, ballname, initial_pose, "world")

def delete_ball(ballname):
    rospy.wait_for_service('gazebo/delete_model')
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(ballname)

def read_SCV():
    with open('/home/denn1slu/Dennis/pingpong_trajectory/30fps_test.csv') as csvfile:
        #rows = csv.reader(csvfile)
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            print(type(float(row[2])))

def read_SCV_spawm():
    with open('/home/denn1slu/Dennis/pingpong_trajectory/curve1_test.csv') as csvfile:
    #with open('/home/denn1slu/Dennis/pingpong_trajectory/30fps_test.csv') as csvfile:
        #rows = csv.reader(csvfile)
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            spawm_ball(row[3], 1.37-float(row[5]), -0.762 + float(row[4]), float(row[6])+0.76)
            #rospy.sleep(0.2)
            
def read_SCV_delete():
    with open('/home/denn1slu/Dennis/pingpong_trajectory/curve1_test.csv') as csvfile:
    #with open('/home/denn1slu/Dennis/pingpong_trajectory/30fps_test.csv') as csvfile:
        #rows = csv.reader(csvfile)
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            delete_ball(row[3])
    

if __name__ == '__main__':
    rospy.init_node('spawn_static_ball', anonymous=True)
    #ball_name = "ball2"
    #spawm_ball(ball_name, -1.0, 0.0, 1.5)
    #rospy.sleep(3.)

    #read_SCV()
    read_SCV_spawm()
    rospy.sleep(10.)
    read_SCV_delete()

    #delete_ball(ball_name)
    
  