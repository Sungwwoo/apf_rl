from datetime import datetime
import numpy as np
import torch
from ppo import PPO
from robot_env import RobotEnv


def init():
    env = RobotEnv()
    state_dim = len(env.observations)
    action_dim = len(env.actions)

    max_ep_len = 500
    max_time_step = int(1e6)
    # update network every (update_time_step)
    update_time_step = 500

    # Hyperparameters
    learning_rate = 5e-5
    epsilon = 0.2
    gamma = 0.99
    epochs = 500

    agent = PPO(state_dim, action_dim, learning_rate, epsilon, gamma, epochs)

    time_step = 0

    while time_step < max_time_step:
        state = env.reset()

        ep_reward = 0

        for t in range(1, max_ep_len + 1):
            # Get Action
            action = agent.get_action(state)
            state, reward, done = env.step(action)

            time_step += 1
            ep_reward += reward

            # save to buffer
            agent.buffer.rewards.append(reward)
            agent.buffer.dones.append(done)

            # agent update
            if time_step % update_time_step == 0:
                agent.update()

            if done:
                break
