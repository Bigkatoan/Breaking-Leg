# Breaking-Leg

This is the code of Breaking Leg project.

In this project, we have an gym environment for autonomous breaking leg car, to setup you need:

```python
import gymnasium as gym
import breaking_leg.rl_goal_environment

env = gym.make('BLEnv-v0')
```

## Observation space
- "observation_rgb": rgb image.
- "observation_depth": depth image.
- "goal": rgb image, contain goal of agent.

## Action space
Action space is (2, ), A[0] is speed, A[1] is angle.

Action space run from -1 to 1.

## Reward
reward return all -1 for every situation, you need to config by yourself using goal and observation_rgb. No terminated!!!

## Basic usage
```python
import gymnasium as gym
import breaking_leg.rl_goal_environment

env = gym.make('BLEnv-v0')
observation, _ = env.reset()
while True:
    action = [0, 0]
    observation, r, t, _, _ = env.step(action)
    if t:
        break
```
