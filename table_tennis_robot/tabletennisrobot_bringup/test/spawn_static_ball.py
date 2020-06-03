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

path = '/home/lab606a/Documents/pingpong_trajectory/irl_trajectory/traj_0512_j1_rand/hit_traj/traj_j1_0512_1.csv'
model_name = 'ball_'


def spawm_ball(ballname, X, Y, Z):
    initial_pose = Pose()
    initial_pose.position.x = X
    initial_pose.position.y = Y
    initial_pose.position.z = Z
    f = open('/home/lab606a/ros_workspace/vscode/src/table_tennis_robot/pingpong_description/models/pingpong_static.sdf','r')
    sdff = f.read()
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(ballname, sdff, ballname, initial_pose, "world")

def delete_ball(ballname):
    rospy.wait_for_service('gazebo/delete_model')
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(ballname)


def read_SCV_spawm():
    with open(path) as csvfile:
    
        #rows = csv.reader(csvfile)
        rows = csv.reader(csvfile, delimiter=',')
        count = 0
        for row in rows:
            count += 1
            spawm_ball(model_name+str(count), float(row[0]), float(row[1]), float(row[2]))
            #rospy.sleep(0.2)
            
def read_SCV_delete():
    with open(path) as csvfile:
    
        #rows = csv.reader(csvfile)
        rows = csv.reader(csvfile, delimiter=',')
        count = 0
        for row in rows:
            count += 1
            delete_ball(model_name+str(count))
    

if __name__ == '__main__':
    rospy.init_node('spawn_static_ball', anonymous=True)
    #ball_name = "ball2"
    #spawm_ball(ball_name, -1.0, 0.0, 1.5)
    #rospy.sleep(3.)

    read_SCV_spawm()
    rospy.sleep(10.)
    read_SCV_delete()

    #delete_ball(ball_name)
    
  