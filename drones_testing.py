from copy import deepcopy
from random import random
import sys
import time
import airsim

import numpy as np



class Point:
    def __init__(self, x, y, z, r, g, b, s=0, e=0) -> None:
        self.location = (x, y, z)
        self.color = (r/255, g/255, b/255)
        self.start, self.end = s, e
    
    def __str__(self) -> str:
        return f'Point(location={str(self.location)}, color={str(self.color)}, start={str(self.start)}, end={str(self.end)}'
    
    def to_list(self) -> list:
        return [[self.location[0], self.location[1], self.location[2]], [self.color[0], self.color[1], self.color[2]]]    

    def to_numpy(self) -> np.ndarray:
        return np.array([[self.location[0], self.location[1], self.location[2]], [self.color[0], self.color[1], self.color[2]]])

class FLSDrone:
    def __init__(self, name, pose, color=[0, 0, 0]) -> None:
        self.name = name
        self.type = 'simpleflight'
        self.pose = airsim.Pose(airsim.Vector3r(pose[0], pose[1], pose[2]), airsim.Quaternionr())
        self.pawn_path = 'FLSDronePawn'
        self.color = color

# def generate_point_cloud_from_frame(frame: np.ndarray):
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(frame[:, 0, :].reshape(-1, 3))
#     pcd.colors = o3d.utility.Vector3dVector(frame[:, 1, :].reshape(-1, 3))
#     return pcd

# bag = rosbag.Bag('./crose115.bag')
# points = []
# for msg in bag.read_messages():
#     params = dict(map(lambda x: (x[0].strip(), x[1].strip()), filter(lambda x: len(x) > 1 and x[1] != '', (map(lambda x: x.strip().split(':'), str(msg.message).split('\n'))))))
#     point = Point(float(params['x']), float(params['y']), float(params['z']), float(params['r']), float(params['g']), float(params['b']), int(params['start']), int(params['end']))
#     points.append(point)

starts = [list(map(lambda x: x.to_list(), filter(lambda x: x.start == i, points))) for i in range(1, 116)]
ends = [list(map(lambda x: x.to_list(), filter(lambda x: x.end == i, points))) for i in range(1, 116)]
from_starts = list(filter(lambda x: len(x[1]) > 0, enumerate(starts)))
from_ends = list(filter(lambda x: len(x[1]) > 0, enumerate(ends)))
frames = list(map(lambda x: (x[0], np.array(x[1])), from_starts + from_ends))
pt_clds = list(map(lambda x: generate_point_cloud_from_frame(x[1]), frames))
down_sampled_pt_clds = [pcd.voxel_down_sample(1) for pcd in pt_clds]

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {color[0]} {color[1]} {color[2]}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'
ORIGIN = (47.641468, -122.140165, 122)

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

drones = []

for i, (pt, cl) in enumerate(zip(np.asarray(down_sampled_pt_clds[0].points).tolist(), (np.asarray(down_sampled_pt_clds[0].colors) * 255).astype(np.int32).tolist())):
    drones.append(FLSDrone(f'fls{i}', [pt[1], pt[0], 50 - pt[2]], cl))
print('ADDED')

for i, drone in enumerate(drones):
    client.simAddVehicle(drone.name, drone.type, airsim.Pose(airsim.Vector3r(3, 3, 0), airsim.Quaternionr()), drone.pawn_path)
    print(client.isApiControlEnabled(drone.name), drone.pose)
    client.enableApiControl(True, drone.name)
    client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, color=drone.color))
    # client.armDisarm(True, drone.name)
    client.takeoffAsync(vehicle_name=drone.name).join()
    client.moveToPositionAsync(drone.pose['position']['x_val'], drone.pose['position']['y_val'], drone.pose['position']['z_val'], 2, vehicle_name=drone.name).join()
    client.hoverAsync(drone.name).join()
    print(i, drone.pose)
    time.sleep(0.05)

print('DONE')
time.sleep(10)

for drone in drones:
    client.simRunConsoleCommand(RESET_COMMAND.format(vehicle_name=drone.name))

# for drone in drones:
#     client.simAddVehicle(drone.name, drone.type, drone.pose, drone.pawn_path)
#     if random() < 0.5:
#         client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=255, g=0, b=0))
#     else:
#         client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=0, g=0, b=255))
#     time.sleep(0.05)

# time.sleep(1)

# for _ in range(20):
#     for drone in drones:
#         if random() < 0.5:
#             client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=255, g=0, b=0))
#         else:
#             client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone.name, r=0, g=0, b=255))
#         time.sleep(0.05)
#     time.sleep(1)

# for drone in drones:
#     client.simRunConsoleCommand(RESET_COMMAND.format(vehicle_name=drone.name))