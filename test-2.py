### Creating the rose illumination
from gettext import translation
import numpy as np
import airsim
import time
from MinDist import MinDist
locations = np.load('0_points.npy')
colors = np.load('0_colors.npy')
CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

translation_vector = np.array([0.0,0.0,0.0])

left_bottom = client.simSetObjectPose('leftbottomcylinder', airsim.Pose(airsim.Vector3r(25, 25, 0)))
right_bottom = client.simSetObjectPose('rightbottomcylinder',airsim.Pose(airsim.Vector3r(0, 25, 0)))
left_top = client.simSetObjectPose('leftopcylinder',airsim.Pose(airsim.Vector3r(25, 0, 0)))
right_top = client.simSetObjectPose('rightopcylinder',airsim.Pose(airsim.Vector3r(0, 0, 0)))
# client.simSetTimeOfDay(True, start_datetime = "2022-12-20 23:59:59", is_start_datetime_dst = False, celestial_clock_speed = 1, update_interval_secs = 60, move_sun = False)
# SCALE = 1
# OFFSET = (0,0,0)
Z_OFFSET = airsim.Vector3r(0,0,0)

left_bottom = client.simGetObjectPose('leftbottomcylinder').position - Z_OFFSET
right_bottom = client.simGetObjectPose('rightbottomcylinder').position - Z_OFFSET
left_top = client.simGetObjectPose('leftopcylinder').position - Z_OFFSET
right_top = client.simGetObjectPose('rightopcylinder').position - Z_OFFSET


print(left_bottom)
dispatcher_location = [left_bottom.to_numpy_array().tolist(),
                       right_bottom.to_numpy_array().tolist(),
                       right_top.to_numpy_array().tolist(),
                       left_top.to_numpy_array().tolist()]

print(dispatcher_location)
mean_center = np.mean(np.array(dispatcher_location), axis=0)
print(mean_center.tolist())


min_dist = MinDist(locations, dispatcher_location )
print(list(map(lambda k: (k[0], len(k[1])), min_dist.items())))


# for idx, (p, c) in enumerate(zip(locations, colors)):
#     drone = f'fls_{idx}'
#     client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(SCALE*p[1], SCALE*p[0], SCALE*(OFFSET[2]-p[2])), airsim.Quaternionr()), 'FLSDronePawn')
#     r, g, b = c
#     client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))

# time.sleep(2)

# start_location = airsim.Pose(airsim.Vector3r(0, 0, 0))
# client.simSetVehiclePose(start_location, True, 'FLSDrone')
# end_location = airsim.Pose(airsim.Vector3r(100, -50, -50))
# xyz = deepcopy(start)
# update_interval = 0.5 if end[0] > start[0] else -0.5
# while xyz != end:
#     xyz += update_interval
#     y,z = val_y_z(start,end, xyz[0])
    
#     xyz[1],xyz[2] = y,z
#     # print(xyz)
#     client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(*xyz)-start_location.position), True, 'FLSDrone')
#     time.sleep(0.1)
#     print(client.simGetVehiclePose('FLSDrone').position)
