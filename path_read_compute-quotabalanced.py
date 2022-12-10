import numpy as np
import airsim
import time
import sys


## Read the path for the drones and dispatcher location
locations = [np.load(f'dispatcher_{i}.npy', allow_pickle=True) for i in range(4)]

CONSOLE_COMMAND = 'ke {vehicle_name} SetLightColor {r} {g} {b}'
  
print(locations[0].shape)
print(locations[1].shape)    


client = airsim.MultirotorClient()
client.enableApiControl(True)

# print(locations)
# print(locations[0].shape)

dispatchers = {0, 1, 2, 3}

left_bottom = client.simSetObjectPose('leftbottomcylinder', airsim.Pose(airsim.Vector3r(25, 30, 0)))
right_bottom = client.simSetObjectPose('rightbottomcylinder',airsim.Pose(airsim.Vector3r(0, 30, 0)))
left_top = client.simSetObjectPose('leftopcylinder',airsim.Pose(airsim.Vector3r(25, 0, 0)))
right_top = client.simSetObjectPose('rightopcylinder',airsim.Pose(airsim.Vector3r(0, 0, 0)))

checkValid = lambda x: x[0] and x[1] and x[2]
end_pos = {}
start_pos = {}
idx, disp = 0, 0
while True:
    for disp in range(4):
        if locations[disp].shape[1] <= idx:
            dispatchers -= {disp}
            continue
        else:
            prev_frame = locations[disp][:, idx-1, :] if idx > 0 else [[None, None, None] for _ in range(locations[disp].shape[0])]
            frame = locations[disp][:, idx, :]
            for point_idx in range(frame.shape[0]):
                drone = f'fls_{(point_idx * 4) + disp}'
                if checkValid(frame[point_idx]):    
                    y, x, z = frame[point_idx]
                    pose = airsim.Vector3r(x, y, -z)
                    if not checkValid(prev_frame[point_idx]):
                        client.simAddVehicle(drone, 'simpleflight', airsim.Pose(pose), 'FLSDronePawn')
                        start_pos[drone] = pose
                    else:
                        client.simSetVehiclePose(airsim.Pose(pose - start_pos[drone]), True, drone)
                        end_pos[drone] = tuple(pose.to_numpy_array().tolist())
    idx += 1
    #time.sleep(0.05)
    if not dispatchers:
        print('DONE')
        break

colors = np.load('0_colors.npy')
points = np.load('0_points.npy')
#client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=drone, r=255 * r, g=255 * g, b=255 * b))
compute_color = {}
for c,p in zip(colors,points):
    y,x,z = p.tolist()
    compute_color[(x,y,-z)] = c

end_pos = {(v[0],v[1],v[2]):k for k,v in end_pos.items()}

print(len(compute_color.keys()))
print(len(end_pos.keys()))

for i in compute_color:
    r,g,b = compute_color[i]
    min_var,closest = np.inf,None
    for j in end_pos:
        d = np.linalg.norm(np.array(j)-np.array(i))
        if d<min_var:
            min_var,closest = d,j
    client.simRunConsoleCommand(CONSOLE_COMMAND.format(vehicle_name=end_pos[closest], r=255 * r, g=255 * g, b=255 * b))

    


        