from collections import deque

import pandas as pd
import numpy as np
import os
import random
from collections import deque
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input, LeakyReLU, BatchNormalization
from keras.layers import Activation
from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam, RMSprop
import keras.backend as K


class ExpertDataSet:
    def __init__(self, expert_traj_path, joint):
        self.expert_dataset = deque()
        self.get_expert_dataset(expert_traj_path)
        self.dataset_num = len(self.expert_dataset)
        print('data set num: {}'.format(self.dataset_num))
        if joint == 'slide':
            print('use slide data set')
            self.expert_dataset = self.get_slide_expert_dataset()
        elif joint == 'j1':
            print('use j1 data set')
            self.expert_dataset = self.get_j1_expert_dataset()
        else:
            print('XXXXXXXXXXXXXXXXXXX')
        self.expert_dataset_shape = np.array(random.sample(self.expert_dataset,1)).shape[1]
        print(self.expert_dataset_shape)
        
        
    def get_expert_dataset(self, expert_traj_path):
        fileList = os.listdir(expert_traj_path)
        fileNum = len(fileList)
        fileCount = 0
        max_traj_step = 18
        expert_dataset = np.zeros((fileNum,max_traj_step,19))
        for i in range(fileNum):
            temp = pd.read_csv(expert_traj_path + fileList[i], sep=',',header=None)
            tmparr = temp.values #(step_num, 19)
            #print(fileList[i] + ' count:{}, type:{}, shape:{}'.format(i+1,type(tmparr),tmparr.shape))
            for j in range(tmparr.shape[0]):
                self.expert_dataset.append(tmparr[j,:])
                
    def get_slide_expert_dataset(self):
        copy = self.expert_dataset.copy()
        for _ in range(self.dataset_num):
            tmparr = copy.pop()
            newarr = np.zeros((4))
            newarr[0] = tmparr[0] / 0.6 #ball x
            newarr[1] = tmparr[1] / 2.0 #ball y
            newarr[2] = tmparr[3] * 2.0 #slide pos
            newarr[3] = tmparr[13]* 2.0 #slide cmd
            copy.appendleft(newarr)
        return copy

    def get_j1_expert_dataset(self):
        copy = self.expert_dataset.copy()
        for _ in range(self.dataset_num):
            tmparr = copy.pop()
            newarr = np.zeros((2))
            newarr[0] = tmparr[1] / 2.0 #ball y
            newarr[1] = tmparr[14] / 2.36 #j1 pos
            #newarr[1] = tmparr[5] / 2.36 #j1 pos
            #newarr[2] = tmparr[14]/ 2.36 #j1 cmd
            copy.appendleft(newarr)
        return copy
            
    '''            
    def get_slide_norm(self):
        self.slide_norm = self.slide_expert_dataset.copy()
        for _ in range(self.dataset_num):
            tmparr = self.slide_norm.pop()
            tmparr[0] = tmparr[0] / 0.6
            tmparr[1] = tmparr[1] / 2
            tmparr[2] = tmparr[2] * 2
            tmparr[3] = tmparr[3] * 2
            self.slide_norm.appendleft(tmparr)

    def get_j1_norm(self):
        self.j1_norm = self.expert_dataset.copy()
        for _ in range(self.dataset_num):
            tmparr = self.j1_norm.pop()
            newarr = np.zeros((3))
            newarr[0] = tmparr[1] / 2.0
            newarr[1] = tmparr[5] / 2.36
            newarr[2] = tmparr[14] / 2.36
            self.j1_norm.appendleft(newarr)
    '''

class Traj_Discriminator(ExpertDataSet):
    def __init__(self, expert_traj_path, joint):
        super().__init__(expert_traj_path=expert_traj_path, joint=joint)
        
        self.D = None   # discriminator
        self.DM = None  # discriminator model
        self.dis_model = self.discriminator_model()
        
        
    def discriminator(self):
        if self.D:
            return self.D
        self.D = Sequential()

        self.D.add(Dense(512, input_dim=self.expert_dataset_shape))
        self.D.add(LeakyReLU(alpha=0.2))
        self.D.add(Dense(512))
        self.D.add(LeakyReLU(alpha=0.2))
        self.D.add(Dropout(0.4))
        self.D.add(Dense(512))
        self.D.add(LeakyReLU(alpha=0.2))
        self.D.add(Dropout(0.4))
        #self.D.add(Dense(1, activation='sigmoid'))
        self.D.add(Dense(1, activation='tanh'))
        '''
        self.D.add(Dense(512, use_bias=False, input_dim=self.expert_dataset_shape))
        self.D.add(BatchNormalization())
        self.D.add(LeakyReLU(alpha=0.2))

        self.D.add(Dense(512, use_bias=False))
        self.D.add(BatchNormalization())
        self.D.add(LeakyReLU(alpha=0.2))
        
        self.D.add(Dense(512, use_bias=False))
        self.D.add(BatchNormalization())
        self.D.add(LeakyReLU(alpha=0.2))

        
        self.D.add(Dense(1, use_bias=False))
        self.D.add(BatchNormalization())
        self.D.add(Activation('sigmoid'))
        '''
        self.D.summary()
        
        return self.D
    
    def discriminator_model(self):
        if self.DM:
            return self.DM
        optimizer = RMSprop(lr=0.0002, decay=6e-8)
        self.DM = Sequential()
        self.DM.add(self.discriminator())
        self.DM.compile(loss='binary_crossentropy', optimizer=optimizer,\
            metrics=['accuracy'])
        return self.DM

    def train(self,states, actions, batch_size=64):
        #states shape (batch_size, 3)
        #actions shape (batch_size, 1)
        expert_dis_input = np.array(random.sample(self.expert_dataset, batch_size))
        policy_dis_input = np.concatenate((states, actions),axis=1)#shape (batch_size, 4)
        
    
        x = np.concatenate((expert_dis_input, policy_dis_input))
        y = np.ones([2*batch_size, 1])
        y[batch_size:, :] = 0
        d_loss = self.dis_model.train_on_batch(x, y)
    
    def get_reward(self, states, actions):
        policy_dis_input = np.concatenate((states, actions),axis=1)
        return self.dis_model.predict(policy_dis_input)

    def save(self, save_path):
        self.dis_model.save_weights(save_path, overwrite=True)
    def load(self, load_path):
        self.dis_model.load_weights(load_path)