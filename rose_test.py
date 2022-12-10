### Creating the rose illumination
import numpy as np
import airsim
import time

locations = np.load('0_points.npy')
colors = np.load('0_colors.npy')
CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
RESET_COMMAND = 'ke {vehicle_name} SetLightColor 0 0 0'

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

left_bottom = client.simSetObjectPose('leftbottomcylinder', airsim.Pose(airsim.Vector3r(25, 30, 0)))
right_bottom = client.simSetObjectPose('rightbottomcylinder',airsim.Pose(airsim.Vector3r(0, 30, 0)))
left_top = client.simSetObjectPose('leftopcylinder',airsim.Pose(airsim.Vector3r(25, 0, 0)))
right_top = client.simSetObjectPose('rightopcylinder',airsim.Pose(airsim.Vector3r(0, 0, 0)))

# client.simSetTimeOfDay(True, start_datetime = "2022-12-20 23:59:59", is_start_datetime_dst = False, celestial_clock_speed = 1, update_interval_secs = 60, move_sun = False)
SCALE = 1
OFFSET = (0,0,0)
total = np.array([0.0, 0.0, 0.0])
for idx, (p, c) in enumerate(zip(locations, colors)):
    drone = f'fls_{idx}'
    client.simAddVehicle(drone, 'simpleflight', airsim.Pose(airsim.Vector3r(SCALE*p[1], SCALE*p[0], SCALE*(OFFSET[2]-p[2])), airsim.Quaternionr()), 'FLSDronePawn')
    r, g, b = c
    client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))
    
for idx in range(locations.shape[0]):
    total += client.simGetVehiclePose(f'fls_{idx}').position.to_numpy_array()

center = total / locations.shape[0]
print(center)


time.sleep(2)
