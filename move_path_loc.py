from copy import deepcopy
from random import random
import sys
import time
import airsim
import math
import numpy as np


# client = airsim.MultirotorClient()
# client.confirmConnection()
# client.enableApiControl(True, 'FLSDrone')
# client.armDisarm(True)

# locations = np.load('0_points.npy')
# colors = np.load('0_colors.npy')


# print(locations[0])
# start = [0,0,0]
# end = [100,-50,-10]

# CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
# client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name='FLSDrone', r=255 , g=255, b=255))
## Equation of Line in 3D

def compute_path(start,end):
    def val_y_z(start,end,x_curr):
        
        y = (start[1] - end[1]) * (x_curr - end[0]) / (start[0] - end[0] ) + end[1]
        z = (start[2] - end[2]) * (y - end[1]) / (start[1] - end[1]) + end[2]
        return y,z

    xyz = deepcopy(start)
    #print(end[0], start[0])
    update_interval = 0.5 if end[0] > start[0] else -0.5
    
    final_path = [start]
    # print(np.linalg.norm(np.array(xyz) - np.array(end), 2))
    # while np.linalg.norm(np.array(xyz) - np.array(end), 2) > 2 :
    while abs((np.array(xyz) - np.array(end))[0]) > 0.5:    
        xyz[0] += update_interval
        y,z = val_y_z(start,end, xyz[0])
        
        xyz[1],xyz[2] = y,z
        final_path.append(deepcopy(xyz))
        
    final_path.append(end.tolist())
    return final_path





