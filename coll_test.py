import sys
import time
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
# client.enableApiControl(True)

drone = 'fls_1'
start_pose = airsim.Vector3r(2, 2, -10)
pose = airsim.Vector3r(2, 2, -10)

client.simAddVehicle(drone, 'simpleflight', airsim.Pose(start_pose), 'FLSDronePawn')
client.simRunConsoleCommand('ke fls_1 SetLightColor 255 0 0')
client.simSetObjectPose('M_Conveyor_BP_2', airsim.Pose(airsim.Vector3r(2,2,0)))
while pose.z_val < 0:
        pose.z_val += 1
        print(pose.to_numpy_array())
        client.simSetVehiclePose(airsim.Pose(pose - start_pose), False, drone)
        time.sleep(1)