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

from std_msgs.msg import Float64





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

def strike(pub,pos):
    moveto = pos
    pub.publish(moveto)

    
    

if __name__ == '__main__':
    rospy.init_node('insert_object_1', anonymous=True)
    j0_pub = rospy.Publisher('/TTbot/slide_rail_joint_position_controller/command', Float64, queue_size=10)
    j1_pub = rospy.Publisher('/TTbot/arm_joint1_position_controller/command', Float64, queue_size=10)
    j2_pub = rospy.Publisher('/TTbot/arm_joint2_position_controller/command', Float64, queue_size=10)
    j3_pub = rospy.Publisher('/TTbot/arm_joint3_position_controller/command', Float64, queue_size=10)
    j4_pub = rospy.Publisher('/TTbot/arm_joint4_position_controller/command', Float64, queue_size=10)
    
    

    ball_name = "ball2"
    spawm_ball(ball_name)

    initpose = Pose()
    initpose.position.x = -1.5
    initpose.position.y = 0.2
    initpose.position.z = 1.0
    inittwist = Twist()
    inittwist.angular.x = 0.0
    inittwist.angular.y = 0.0
    inittwist.angular.z = 0.0
    inittwist.linear.x = 5.0
    inittwist.linear.y = 0.0
    inittwist.linear.z = 2.0

    rospy.sleep(1.)
    strike(j0_pub,0.33)
    strike(j1_pub,-1.9)
    strike(j2_pub,-1.2)
    strike(j3_pub,1.0)
    strike(j4_pub,1.0)
    rospy.sleep(1.)
    set_ball_state(ball_name, initpose, inittwist)


    
    ball_pose_z=0.78
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if(ball_pose_z < 0.2):
            break
        
        ball_pose, ball_twist = get_ball_state(ball_name)
        
        if(ball_pose.position.x > 1.4):
            strike(j1_pub,-0.9)
            strike(j2_pub,-1.0)
            
            
        #ball_pose.position.y = ball_pose.position.y + 0.1
        #set_ball_state(ball_name, ball_pose, ball_twist)
        
        ball_pose_z = ball_pose.position.z
        #rospy.loginfo(ball_pose_z)
        rate.sleep()

    delete_ball(ball_name)
    
  