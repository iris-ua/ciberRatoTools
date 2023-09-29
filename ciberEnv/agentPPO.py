
from stable_baselines3 import PPO
import numpy as np
import argparse

import sys
sys.path.append('../pClient')
from croblink import *

SIM_IP = "127.0.0.1"
SIM_PORT = 6000

parser = argparse.ArgumentParser()
parser.add_argument('-m','--model', help='model filename', default='ciber_ppo_3')
parser.add_argument('-s','--server', help='simulator address', default='localhost')

args = parser.parse_args()

model = PPO.load(args.model)

rob = CRobLink("agentPPO", 0, args.server)

action = np.array([0.0,0.0])
while True:
    rob.readSensors()

    obsl = [float(x) for x in rob.measures.lineSensor]
    #obs = np.append(np.array(obsl),action)
    obs = obsl

    action, _states = model.predict(obs, deterministic=True)

    rob.driveMotors(action[0], action[1])

