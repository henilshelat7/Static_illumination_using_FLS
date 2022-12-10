from copy import deepcopy
from random import random
import sys
import time
import airsim

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'
NUMBER_DRONES = 250

class FLSDrone:
    def __init__(self, name, pose, color=(0, 0, 0)) -> None:
        self.name = name
        self.type = 'simpleflight'
        self.pose = airsim.Pose(airsim.Vector3r(pose[0], pose[1], pose[2]), airsim.Quaternionr())
        self.pawn_path = 'FLSDronePawn'
        self.color = color

client = airsim.MultirotorClient(ip=sys.argv[1])
client.confirmConnection()
client.enableApiControl(True)

drone_names = [f'fls{i}' for i in range(NUMBER_DRONES)]
drone_poses = []

curr_pose = [0, 0, 0]
x_inc = 3
y_inc = 3

for i in range(NUMBER_DRONES):
    if i % 10 == 0:
        curr_pose[0] = 0
        curr_pose[1] += y_inc
    else:
        curr_pose[0] += x_inc
    drone_poses.append(deepcopy(curr_pose))

drones = [FLSDrone(n, p) for n, p in zip(drone_names, drone_poses)]
for drone in drones:
    client.simAddVehicle(drone.name, drone.type, drone.pose, drone.pawn_path)
    if random() < 0.5:
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=255, g=0, b=0))
    else:
        client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=0, g=0, b=255))
    time.sleep(0.05)

time.sleep(1)

for _ in range(20):
    for drone in drones:
        if random() < 0.5:
            client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=255, g=0, b=0))
        else:
            client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=0, g=0, b=255))
        time.sleep(0.05)
    time.sleep(1)

for drone in drones:
    client.simRunConsoleCommand(RESET_COMMAND.format(vehicle_name=drone.name))