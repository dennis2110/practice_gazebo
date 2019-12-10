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





def spawm_ball(ballname):
    initial_pose = Pose()
    initial_pose.position.x = -1
    initial_pose.position.y = 0
    initial_pose.position.z = 2

    f = open('/home/denn1slu/ros_workspace/vscode_ws/src/table_tennis_robot/pingpong_description/models/pingpong.sdf','r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(ballname, sdff, ballname, initial_pose, "world")

def delete_ball(ballname):
    rospy.wait_for_service('gazebo/delete_model')
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(ballname)

def get_ball_state(ballname):
    rospy.wait_for_service('gazebo/get_model_state')
    get_ball_state_prox = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    ball_state_resp = get_ball_state_prox(ballname,"")
    return ball_state_resp.pose, ball_state_resp.twist

def set_ball_state(ballname,ballpose,balltwist):
    ball_model_state = ModelState()
    ball_model_state.model_name = ballname
    ball_model_state.pose = ballpose
    ball_model_state.twist = balltwist
    ball_model_state.reference_frame = "world"
    rospy.wait_for_service('gazebo/set_model_state')
    set_ball_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    set_ball_state_prox(ball_model_state)
    
    

if __name__ == '__main__':
    rospy.init_node('insert_object_1', anonymous=True)
    ball_name = "ball2"
    spawm_ball(ball_name)

    initpose = Pose()
    initpose.position.x = -2.0
    initpose.position.y = 0.0
    initpose.position.z = 1.5

    inittwist = Twist()
    inittwist.angular.x = 0.0
    inittwist.angular.y = 0.0
    inittwist.angular.z = -10.0
    inittwist.linear.x = 2.0
    inittwist.linear.y = 0.0
    inittwist.linear.z = 5.0
    set_ball_state(ball_name, initpose, inittwist)


    ball_pose_z=0.78
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        if(ball_pose_z < 0.75):
            break

        ball_pose, ball_twist = get_ball_state(ball_name)
        
        ball_pose.position.y = ball_pose.position.y + 0.1
        set_ball_state(ball_name, ball_pose, ball_twist)
        
        ball_pose_z = ball_pose.position.z
        rospy.loginfo(ball_pose_z)
        rate.sleep()

    delete_ball(ball_name)
    
  