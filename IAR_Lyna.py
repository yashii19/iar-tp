# -*- coding: utf-8 -*-
"""
Created on Sat Oct  8 19:56:29 2022

@author: Lyna
"""


import gym

import numpy as np
from stable_baselines3 import PPO, A2C, SAC, TD3, DQN

from stable_baselines3.common.base_class import BaseAlgorithm

#--------------------------------------

def evaluate_agent(
    agent: BaseAlgorithm,
    env: gym.Env,
    deterministic: bool = False,
) -> float:
    """
    Evaluate an RL agent for 1 episode.

    :param model: the RL Agent
    :param env: the gym Environment
    :param deterministic: Whether to use deterministic or stochastic actions
    :return: Mean reward for the episode.
    """

    # Retrieve first observation
    # To be completed
    obs= env.reset()
    done = False
    total_reward=0

    while not done:
      # The agent predicts the action to take given the observation
      action, _ = agent.predict(obs, deterministic)

      # Check that predict is properly used: we use discrete actions,
      # therefore `action` should be an int here
      assert env.action_space.contains(action)

      # The environment performs a step and produces the next state, the reward
      # and whether the episode is over. The info return is a placeholder for
      # any supplementary information that one may need.
      obs, reward, done, info = env.step(action)
      
      # The total reward over the episode is the sum of rewards at each step
      # no discount here, discount is used in the reinforcement learning process
      total_reward += reward
      
    return total_reward

#--------------------------------------------------------------------
  
class RewardPrinterWrapper(gym.Wrapper):
  """
  :param env:  Gym environment that will be wrapped
  """
  def __init__(self, env: gym.Env):
    # Call the parent constructor, so we can access self.env later
    super().__init__(env)
  
  def reset(self):
    """
    Reset the environment 
    """
    obs = self.env.reset()
    return obs

  def step(self, action):
    """
    :param action: ([float] or int) Action taken by the agent
    :return: (np.ndarray, float, bool, dict) observation, reward, is the episode over?, additional informations
    """
    obs, reward, done, infos = self.env.step(action)
    print(reward)
    return obs, reward, done, infos


# ---------------------------------------------------------------------
    # MAIN

# Create the gym Env
env = gym.make('LunarLander-v2')
#env.reset()
#env.render()

# Wrap the RewardPRinterWrapper around it
env = RewardPrinterWrapper(env)

# Create the RL agent
agent = PPO("MlpPolicy", env)

# Call the interaction loop
reward = evaluate_agent(agent, env)

# Print the final reward
print(reward)