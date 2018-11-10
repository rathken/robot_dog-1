################################################################################
# Copyright (c) 2018 Chi K. Lee
# Release under BSD 3-Clause License 
################################################################################
import gym
from baselines import deepq
import quadruped

def callback(lcl, glb):
    # stop training if reward exceeds 20
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 20
    return is_solved

def main():
    # create the environment
    env = gym.make("quadruped-v0")
    # create the learning agent
    model = deepq.models.mlp([1000,1000,1000,500,250])

    # train the agent on the environment
    act = deepq.learn( env, q_func=model,
                       lr=1e-3,
                       max_timesteps=2000000,
                       buffer_size=50000,
                       exploration_fraction=0.1,
                       exploration_final_eps=0.01,
                       print_freq=10,
                       callback=callback
                       )
    # save trained model
    act.save("qaudruped.pkl")

    
if __name__ == '__main__':
    main()


    
