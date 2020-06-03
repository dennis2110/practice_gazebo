#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from openai_ros.msg import RLExperimentInfo
import matplotlib
import matplotlib.pyplot as plt
#import torch

# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display

ep_number = 0
ep_reward  = 0
episode_durations = []
started = False

def RLinfo_callback(data):
    global ep_number, ep_reward, started,episode_durations
    #global episode_durations
    if (not started):
        started = True
    ep_number = data.episode_number
    ep_reward = data.episode_reward
    episode_durations.append(ep_reward)
    #print(ep_number, ep_reward)
    print('episode {} reward is : {}'.format(ep_number, ep_reward))
    #print(episode_durations)
    plot_durations()
    
    

def RLinfo_listener():
    rospy.init_node('RL_listen', anonymous=True)
    rospy.Subscriber("/openai/reward/irl", RLExperimentInfo, RLinfo_callback)
    #rospy.spin()

def plot_durations():
    plt.figure(1)
    plt.clf()
    #durations_t = torch.tensor(episode_durations, dtype=torch.float)
    array=np.array(episode_durations)
    plt.title('Training...')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    #plt.plot(durations_t.numpy())
    plt.plot(array)
    #if len(durations_t) >= 100:
    #    means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
    #    means = torch.cat((torch.zeros(99), means))
    #    plt.plot(means.numpy())
    
    plt.pause(0.001)
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())  

#def timer_callback(event):
#    global started
#    if (started):
#        #print(episode_durations)
#        print "Last message published"



if __name__ == '__main__':
    plt.ion()
    RLinfo_listener()
    #timer = rospy.Timer(rospy.Duration(0.5), timer_callback)

    rospy.spin()    
    #timer.shutdown()
    '''
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        print('bbbbbbbbbbbbbbb')
        print(episode_durations)
        if(is_listing == True):
            print('cccccccccccccc')
            plot_durations()
        #rospy.spin()
        rate.sleep()
        

    print('Complete')
    plt.ioff()
    plt.show()
    '''
