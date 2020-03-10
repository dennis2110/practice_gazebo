#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import torch

rand_value = [12,26,3,17,20,36,7,9,21,19]
episode_durations = []

# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display

def init_rosnode():
    rospy.init_node('dynamic_plot',anonymous=True)
    #rospy.spin()

def plot_durations():
    plt.figure(2)
    plt.clf()
    print(episode_durations)
    #durations_t = torch.tensor(episode_durations, dtype=torch.float)
    array=np.array(episode_durations)
    plt.title('Training...')
    plt.xlabel('Episode')
    plt.ylabel('Duration')
    plt.plot(array)
    
    plt.pause(0.5)  # pause a bit so that plots are updated
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())  

if __name__ == '__main__':
    plt.ion()
    init_rosnode()

    num_t = 10

    for t in range(num_t):
        episode_durations.append(rand_value[t])
        plot_durations()

    print('Complete')
    plt.ioff()
    plt.show()


