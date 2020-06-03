#import tensorflow as tf
import numpy as np
import tensorflow as tf

from keras.layers import Input, Dense, Activation, BatchNormalization
from keras.layers.merge import Add, Concatenate
from keras.models import Sequential, Model
from keras.optimizers import Adam


class ActorCritic:
    def __init__(self, env, sess, joint):
        self.env  = env
        self.sess = sess

        self.learning_rate = 0.0001
        self.epsilon = .9
        #self.epsilon_decay = .99995
        self.epsilon_decay = .995
        self.gamma = .90
        self.tau   = .01
        self.action_space = np.array([0.0])

        if joint == 'slide':
            self.obs_space = np.zeros((3))
        elif joint == 'j1':
            self.obs_space = np.zeros((1))
        else:
            self.obs_space = 0
        

        # ===================================================================== #
        #                               Actor Model                             #
        # Chain rule: find the gradient of chaging the actor network params in  #
        # getting closest to the final value network predictions, i.e. de/dA    #
        # Calculate de/dA as = de/dC * dC/dA, where e is error, C critic, A act #
        # ===================================================================== #

        #self.memory = deque(maxlen=4000)
        
        self.actor_state_input, self.actor_model = self.create_actor_model()
        _, self.target_actor_model = self.create_actor_model()

        self.actor_critic_grad = tf.placeholder(tf.float32,
            [None, self.action_space.shape[0]]) # where we will feed de/dC (from critic)

        actor_model_weights = self.actor_model.trainable_weights
        self.actor_grads = tf.gradients(self.actor_model.output,
            actor_model_weights, -self.actor_critic_grad) # dC/dA (from actor)
        grads = zip(self.actor_grads, actor_model_weights)
        self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(grads)

        # ===================================================================== #
        #                              Critic Model                             #
        # ===================================================================== #

        self.critic_state_input, self.critic_action_input, \
            self.critic_model = self.create_critic_model()
        _, _, self.target_critic_model = self.create_critic_model()


        self.critic_grads = tf.gradients(self.critic_model.output,
            self.critic_action_input) # where we calcaulte de/dC for feeding above

        # Initialize for later gradient calculations
        #self.sess.run(tf.initialize_all_variables())
        self.sess.run(tf.global_variables_initializer())
        

    # ========================================================================= #
    #                              Model Definitions                            #
    # ========================================================================= #

    def create_actor_model(self):
        state_input = Input(shape=self.obs_space.shape)

        h1 = Dense(50, activation='relu')(state_input)
        h2 = Dense(100, activation='relu')(h1)
        h3 = Dense(50, activation='relu')(h2)
        output = Dense(self.action_space.shape[0], activation='tanh')(h3)
        '''
        h1 = Dense(50, use_bias=False)(state_input)
        bn1= BatchNormalization()(h1)
        a1 = Activation('relu')(bn1)

        h2 = Dense(100, use_bias=False)(a1)
        bn2= BatchNormalization()(h2)
        a2 = Activation('relu')(bn2)


        h3 = Dense(50, use_bias=False)(a2)
        bn3= BatchNormalization()(h3)
        a3 = Activation('relu')(bn3)


        output = Dense(self.action_space.shape[0], use_bias=False)(a3)
        output = BatchNormalization()(output)
        output = Activation('tanh')(output)
        '''

        model = Model(input=state_input, output=output)
        adam  = Adam(lr=0.0001)
        model.compile(loss="mse", optimizer=adam)
        return state_input, model

    def create_critic_model(self):
        state_input = Input(shape=self.obs_space.shape)

        state_h1 = Dense(50, activation='relu')(state_input)
        state_h2 = Dense(100)(state_h1)

        #action_input = Input(shape=self.env.action_space.shape)
        action_input = Input(shape=self.action_space.shape)
        action_h1    = Dense(50)(action_input)

        merged    = Concatenate()([state_h2, action_h1])
        merged_h1 = Dense(50, activation='relu')(merged)
        output = Dense(1, activation='linear')(merged_h1)
        '''
        state_h1 = Dense(50, use_bias=False)(state_input)
        state_bn1= BatchNormalization()(state_h1)
        state_a1 = Activation('relu')(state_bn1)

        state_h2 = Dense(100, use_bias=False)(state_a1)
        state_bn2= BatchNormalization()(state_h2)
        

        
        action_input = Input(shape=self.action_space.shape)

        action_h1 = Dense(50, use_bias=False)(action_input)
        action_bn1= BatchNormalization()(action_h1)

        merged    = Concatenate()([state_bn2, action_bn1])
        merged_h1 = Dense(50, use_bias=False)(merged)
        merged_bn1= BatchNormalization()(merged_h1)
        merged_a1 = Activation('relu')(merged_bn1)

        output = Dense(1, use_bias=False)(merged_a1)
        output = BatchNormalization()(output)
        output = Activation('linear')(output)
        '''

        model  = Model(input=[state_input,action_input], output=output)
        adam  = Adam(lr=0.0001)
        model.compile(loss="mse", optimizer=adam)
        return state_input, action_input, model

    # ========================================================================= #
    #                               Model Training                              #
    # ========================================================================= #

    def remember(self, cur_state, action, reward, new_state, done):
        memory.append([cur_state, action, reward, new_state, done])

    def _train_actor(self, samples):
        
            cur_states, actions, rewards, new_states, _ =  stack_samples(samples)
            predicted_actions = self.actor_model.predict(cur_states)
            grads = self.sess.run(self.critic_grads, feed_dict={
                self.critic_state_input:  cur_states,
                self.critic_action_input: predicted_actions
            })[0]

            self.sess.run(self.optimize, feed_dict={
                self.actor_state_input: cur_states,
                self.actor_critic_grad: grads
            })

    def _train_critic(self, samples):

        cur_states, actions, rewards, new_states, dones = stack_samples(samples)
        target_actions = self.target_actor_model.predict(new_states)
        future_rewards = self.target_critic_model.predict([new_states, target_actions])
        #print('dones : {}'.format(dones))
        rewards += self.gamma * future_rewards * (1 - dones)
        
        evaluation = self.critic_model.fit([cur_states, actions], rewards, verbose=0)
        #print(evaluation.history)
    def train(self):
        batch_size = 64
        if len(memory) < batch_size:
            return

        rewards = []
        samples = random.sample(memory, batch_size)
        self.samples = samples
        self._train_critic(samples)
        self._train_actor(samples)

    def train_batch(self, cur_states, actions, rewards, new_states, dones, batch_size=64):
        
        target_actions = self.target_actor_model.predict(new_states)
        future_rewards = self.target_critic_model.predict([new_states, target_actions])
        #print('dones : {}'.format(dones))
        rewards += self.gamma * future_rewards * (1 - dones)
        evaluation = self.critic_model.fit([cur_states, actions], rewards, verbose=0)

        predicted_actions = self.actor_model.predict(cur_states)
        grads = self.sess.run(self.critic_grads, feed_dict={
            self.critic_state_input:  cur_states,
            self.critic_action_input: predicted_actions
        })[0]
        self.sess.run(self.optimize, feed_dict={
            self.actor_state_input: cur_states,
            self.actor_critic_grad: grads
        })


    # ========================================================================= #
    #                         Target Model Updating                             #
    # ========================================================================= #

    def _update_actor_target(self):
        actor_model_weights  = self.actor_model.get_weights()
        actor_target_weights = self.target_actor_model.get_weights()
        
        for i in range(len(actor_target_weights)):
            actor_target_weights[i] = actor_model_weights[i]*self.tau + actor_target_weights[i]*(1-self.tau)
        self.target_actor_model.set_weights(actor_target_weights)

    def _update_critic_target(self):
        critic_model_weights  = self.critic_model.get_weights()
        critic_target_weights = self.target_critic_model.get_weights()
        
        for i in range(len(critic_target_weights)):
            critic_target_weights[i] = critic_model_weights[i]*self.tau + critic_target_weights[i]*(1-self.tau)
        self.target_critic_model.set_weights(critic_target_weights)

    def update_target(self):
        self._update_actor_target()
        self._update_critic_target()


    # ========================================================================= #
    #                              Model Predictions                            #
    # ========================================================================= #

    def act(self, cur_state):
        self.epsilon *= self.epsilon_decay
        if np.random.random() < self.epsilon:
            return (self.actor_model.predict(cur_state) + np.random.normal(scale=0.3))/2
        return self.actor_model.predict(cur_state)

    def act_run(self, cur_state):
        return self.actor_model.predict(cur_state)

    def save_model(self, save_actor_path, save_critic_path):
        self.actor_model.save_weights(save_actor_path, overwrite=True)
        self.critic_model.save_weights(save_critic_path, overwrite=True)

    def load_model(self, load_actor_path, load_critic_path):
        self.actor_model.load_weights(load_actor_path)
        self.target_actor_model.load_weights(load_actor_path)
        self.critic_model.load_weights(load_critic_path)
        self.target_critic_model.load_weights(load_critic_path)