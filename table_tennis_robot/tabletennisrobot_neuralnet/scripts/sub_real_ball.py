#!/usr/bin/env python
import rospy
import numpy as np
import tensorflow as tf
import keras.backend as K

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import JointState

from openai_ros.task_envs.moving_ttbot import inverse_arm
from ttbot_model.discriminator_net import Traj_Discriminator
from ttbot_model.policy_net import ActorCritic


slide_expert_traj_path = "/home/lab606a/Documents/pingpong_trajectory/irl_trajectory/traj_0508_slide_rand/hit_traj/"
j1_expert_traj_path = "/home/lab606a/Documents/pingpong_trajectory/irl_trajectory/traj_0512_j1_rand/hit_traj/"

load_slide_actor_path = "/home/lab606a/ros_workspace/catkin_ws/src/ttbot_tests/models/0512v3-actor-slide-rand-256.h5"
load_slide_critic_path= "/home/lab606a/ros_workspace/catkin_ws/src/ttbot_tests/models/0512v3-critic-slide-rand-256.h5"
load_slide_dis_path   = "/home/lab606a/ros_workspace/catkin_ws/src/ttbot_tests/models/0512v3-discri-slide-rand-256.h5"

load_j1_actor_path = '/home/lab606a/ros_workspace/catkin_ws/src/ttbot_tests/models/0515-j1-actor-v3.h5'
load_j1_critic_path= '/home/lab606a/ros_workspace/catkin_ws/src/ttbot_tests/models/0515-j1-critic-v3.h5'
load_j1_dis_path   = '/home/lab606a/ros_workspace/catkin_ws/src/ttbot_tests/models/0515-j1-disc-v3.h5'



class Get_state():
    def __init__(self):
        rospy.Subscriber("/TTbot/hitting_point", Float32MultiArray, self.ballxyz_callback)
        #rospy.Subscriber("/TTbot/joint_states", JointState, self.realTTbot_state_callback)
        rospy.Subscriber("/realTTbot/motor_status", JointState, self.realTTbot_state_callback)
        
        self.state = np.zeros((7))

    def ballxyz_callback(self, msg):
        self.state[0] = msg.data[0]/100 -0.762 #ball_x
        self.state[1] = msg.data[1]/100 -1.37 #ball_y
        if (msg.data[3]==-10.0):
            self.state[2] = self.state[0]
            self.state[3] = -1.75
            self.state[4] = 1.09
        else:
            self.state[2] = msg.data[3]/100 -0.762 #hit_ball_x
            self.state[3] = msg.data[4]/100 -1.37 #hit_ball_y
            self.state[4] = msg.data[5]/100 +0.76 + 0.09 #hit_ball_z
        
    def realTTbot_state_callback(self, msg):
        self.state[5] = msg.position[0] -0.41 #slide_pos
        #self.state[6] = msg.position[1] #j1_pos
        
class Pub_joint():
    def __init__(self):
        self.slide_pub = rospy.Publisher('/TTbot/slide_rail_joint_position_controller/command', Float64, queue_size=1)
        self.arm1_pub = rospy.Publisher('/TTbot/arm_joint1_position_controller/command', Float64, queue_size=1)
        self.arm2_pub = rospy.Publisher('/TTbot/arm_joint2_position_controller/command', Float64, queue_size=1)
        self.arm3_pub = rospy.Publisher('/TTbot/arm_joint3_position_controller/command', Float64, queue_size=1)
        self.arm4_pub = rospy.Publisher('/TTbot/arm_joint4_position_controller/command', Float64, queue_size=1)
        self.arm5_pub = rospy.Publisher('/TTbot/arm_joint5_position_controller/command', Float64, queue_size=1)
        self.joint_cmd = Float64()
        self.save_pos_pub = rospy.Publisher('/TTbot/save_joint_pos', Bool, queue_size=1)
        self.save_msg = Bool()
    
    def set_pos(self, joint_cmds):
        self.joint_cmd.data = joint_cmds[0,0]
        self.slide_pub.publish(self.joint_cmd)

        self.joint_cmd.data = joint_cmds[0,1]
        self.arm1_pub.publish(self.joint_cmd)

        self.joint_cmd.data = joint_cmds[0,2]
        self.arm2_pub.publish(self.joint_cmd)

        self.joint_cmd.data = joint_cmds[0,3]
        self.arm3_pub.publish(self.joint_cmd)

        self.joint_cmd.data = joint_cmds[0,4]
        self.arm4_pub.publish(self.joint_cmd)

        self.joint_cmd.data = joint_cmds[0,5]
        self.arm5_pub.publish(self.joint_cmd)

    def save_joint_state(self, signal = False):
        self.save_msg.data = signal
        self.save_pos_pub.publish(self.save_msg)

class Pub_ball():
    def __init__(self):
        self.set_ball_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.ball_initpose = Pose()
        self.ball_inittwist = Twist()

    def set_ball_state(self, ball_x, ball_y, ball_z):
        ball_model_state = ModelState()
        ball_model_state.model_name = "ball"

        self.ball_initpose.position.x = ball_x
        self.ball_initpose.position.y = ball_y
        self.ball_initpose.position.z = ball_z

        ball_model_state.pose = self.ball_initpose
        ball_model_state.twist = self.ball_inittwist
        ball_model_state.reference_frame = "world"
        
        rospy.logwarn("START set ball")
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            self.set_ball_state_prox(ball_model_state)
            rospy.logerr("call set ball service")
        except rospy.ServiceException as e:
            print ("/gazebo/set_model_state service call failed")
            
        rospy.logwarn("FINISH set ball")

    

if __name__ == '__main__':
    rospy.init_node('sub_real_ball',anonymous=True)

    pub_pingpong = Pub_ball()
    TTbot_env = Get_state()
    TTbot_step = Pub_joint()
    inversearm = inverse_arm.InverseArm()

    env = None
    sess = tf.Session()
    K.set_session(sess)

    slide_policy = ActorCritic(env, sess, 'slide')
    slide_dis = Traj_Discriminator(slide_expert_traj_path, 'slide')
    j1_policy = ActorCritic(env, sess, 'j1')
    j1_dis = Traj_Discriminator(j1_expert_traj_path, 'j1')

    slide_policy.load_model(load_slide_actor_path, load_slide_critic_path)
    slide_dis.load(load_slide_dis_path)
    j1_policy.load_model(load_j1_actor_path, load_j1_critic_path)
    j1_dis.load(load_j1_dis_path)

    j0_policy_state      = np.zeros((1,slide_policy.obs_space.shape[0]))
    j1_policy_state      = np.zeros((1,j1_policy.obs_space.shape[0]))
    
    action = np.array([[0.0, 1.1, 0.171, -1.825, -0.426, 0.15]])
    strike_point_x = 0.0
    strike_point_y = 1.0
    strike_point_z = 1.09

    loop_count = 0
    get_init_state = False
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        #print(TTbot_env.state)
        
        #ball_y > 1.4
        if (TTbot_env.state[0]>0.3 and TTbot_env.state[0]<0.6) and (TTbot_env.state[1] > 1.4) and (not get_init_state):
            get_init_state = True
            TTbot_step.save_joint_state(True)
        if loop_count > 30 and get_init_state:
            get_init_state = False
            TTbot_step.save_joint_state(False)
            print("stop policy")

        if get_init_state:
            print("in policy")
            #main policy loop
            #print(TTbot_env.state[5])
            j0_policy_state[0,0] = TTbot_env.state[0] / 0.6
            j0_policy_state[0,1] = TTbot_env.state[1] / 2.0
            j0_policy_state[0,2] = TTbot_env.state[5] * 2.0
            j1_policy_state[0,0] = TTbot_env.state[1] / 2.0
            strike_point_x = TTbot_env.state[2]
            strike_point_y = TTbot_env.state[3]
            strike_point_z = TTbot_env.state[4]
            
            slide_action = slide_policy.act_run(j0_policy_state)
            action[0,0] = slide_action[0,0] / 2.0
            action[0,0] = np.clip(action[0,0], -0.5, 0.5)

                    
            j1_action = j1_policy.act_run(j1_policy_state)
            action[0,1] = j1_action[0,0] * 2.36 #+0.35
            action[0,1] = np.clip(action[0,1], -2.36, 2.36)

            if abs(action[0,0] - strike_point_x) > 0.6 or abs(action[0,0] - strike_point_x) < 0.3:
                pass
            else:
                action[0,2], action[0,3], action[0,4] = inversearm.compute_cmd(strike_point_x, strike_point_y, strike_point_z, action[0,0])

            loop_count += 1
        else:
            action[0,0] = 0.0
            action[0,1] = 1.1
            action[0,2] = 0.171
            action[0,3] = -1.825
            action[0,4] = -0.426
            action[0,5] = 0.15
            loop_count = 0
            
        TTbot_step.set_pos(action)


        '''
        #if get ball
        if pub_pingpong.ballxyz[0] == 1.0:
            ball_x = pub_pingpong.ballxyz[1]/100 -0.762
            ball_y = pub_pingpong.ballxyz[2]/100 -1.37
            ball_z = pub_pingpong.ballxyz[3]/100 +0.76
            pub_pingpong.ball_initpose.position.x = ball_x
            pub_pingpong.ball_initpose.position.y = ball_y
            pub_pingpong.ball_initpose.position.z = ball_z
            pub_pingpong.set_ball_state()

        '''

        
        #print(type(pub_pingpong.ballxyz[0]))
        #rospy.spin()
        rate.sleep()



