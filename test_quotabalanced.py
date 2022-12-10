from asyncore import dispatcher
import numpy as np
import airsim
import time
import sys
from QuotaBalanced import QuotaBalanced


## Read the path for the drones and dispatcher location
locations = [np.load(f'test_{i}.npy', allow_pickle=True) for i in range(4)]

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'

client = airsim.MultirotorClient()
client.enableApiControl(True)

# print(locations)
# print(locations[0].shape)

dispatchers = {0, 1, 2, 3}
dispatcher_loc = [(25, 30, 0),(0,30,0),(25,0,0),(0,0,0)]

left_bottom = client.simSetObjectPose('leftbottomcylinder', airsim.Pose(airsim.Vector3r(25, 30, 0)))
right_bottom = client.simSetObjectPose('rightbottomcylinder',airsim.Pose(airsim.Vector3r(0, 30, 0)))
left_top = client.simSetObjectPose('leftopcylinder',airsim.Pose(airsim.Vector3r(25, 0, 0)))
right_top = client.simSetObjectPose('rightopcylinder',airsim.Pose(airsim.Vector3r(0, 0, 0)))

locations = np.load('0_points.npy')
qb, recompute = QuotaBalanced(locations[:, 0, :], dispatcher_loc)

