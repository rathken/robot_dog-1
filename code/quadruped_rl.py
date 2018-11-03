import gym
from baselines import deepq
import quadruped


def main():
    # create the environment
    env = gym.make("quadruped-v0")
    # create the learning agent
    model = deepq.models.mlp([16,16])

if __name__ == '__main__':
    main()


    
