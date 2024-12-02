import gymnasium as gym
import numpy as np
import subprocess
import socket
import time

import sys
sys.path.append('../pClient')
import croblink

SIM_IP = "127.0.0.1"
SIM_PORT = 6000

class CiberEnv1(gym.Env):
    def __init__(self, obs_space, act_space, sim_args) -> None:
        super().__init__()

        self.observation_space=obs_space

        self.action_space=act_space

        self.sim_proc = subprocess.Popen(['../simulator/simulator', *sim_args])

        time.sleep(5)

        self.prev_score = 0
        
    def step(self, action):

        raise NotImplementedError


    def reset(self):
        if(hasattr(self,"agentapi") ):
            self.agentapi.reset()
        else: 
            self.agentapi = croblink.CRobLink('ciberEnv',1,SIM_IP)

        self.agentapi.readSensors()

        self.prev_score = 0

    def close(self):
        self.sim_proc.terminate()
