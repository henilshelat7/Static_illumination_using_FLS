# # ready to run example: PythonClient/multirotor/hello_drone.py
# import airsim
import os
import time
import numpy as np
#import open3d as o3d
from MinDist import MinDist
from QuotaBalanced import QuotaBalanced

# # connect to the AirSim simulator
# client = airsim.MultirotorClient()
# client.confirmConnection()
# client.enableApiControl(True,'FLSDrone')
# client.enableApiControl(True,'FLSDrone-2')
# client.armDisarm(True, 'FLSDrone')
# client.armDisarm(True, "FLSDrone-2")


# consoleCommand = "ke {name} SetLightColor {r} {g} {b}"

# start_location = airsim.Pose(airsim.Vector3r(0, 0, 10))
# client.simSetVehiclePose(start_location, ignore_collision=True)
# time.sleep(5)
# start_location = airsim.Pose(airsim.Vector3r(0, 0, 20))
# client.simSetVehiclePose(start_location, ignore_collision=True)

# client.simRunConsoleCommand(consoleCommand.format(name = 'FLSDrone', r = 255, g= 0, b=0))
# time.sleep(10)
# client.simRunConsoleCommand(consoleCommand.format(name = 'FLSDrone', r = 0, g= 255, b=0))

dispatcher_loc = [[25, 25, 0], [0, 25, 0], [25, 0, 0], [0, 0, 0]]

# dispatcher_loc = [[50.0, 50.0, -0.0], [-50.0, 50.0, -0.0], [-50.0, -50.0, -0.0], [50.0, -50.0, -0.0]]
locations = np.load('0_points.npy')
# locations = np.load('0.npy')

# pt_cloud = o3d.geometry.PointCloud()
# pt_cloud.points = locations[:,0,:]
# pt_cloud.colors = locations[:,1,:]
# print(pt_cloud)

min_dist = MinDist(locations, dispatcher_loc)
print(list(map(lambda k: (k[0], len(k[1])), min_dist.items())))
print(min_dist[0])
# min_dist = MinDist(locations[:, 0, :], dispatcher_loc)
# print(list(map(lambda k: (k[0], len(k[1])), min_dist.items())))

qb, recompute = QuotaBalanced(locations, dispatcher_loc)
#print(list(map(lambda k: (k[0], len(k[1])), qb.items())))
# print('# recomputes:', recompute)

# qb, recompute = QuotaBalanced(locations[:, 0, :], dispatcher_loc, 5, 5)
# print(list(map(lambda k: (k[0], len(k[1])), qb.items())))
# print('# recomputes:', recompute)