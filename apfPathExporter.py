import time

import numpy as np
import airsim
import tqdm

from APF import APFPathPlanner
from Client import Client
from stag import StagOrchestrator
from PointCloud import PointCloud

from QuotaBalanced_APF import QuotaBalanced

def place_objects(client: Client):
    left_bottom = client.simSetObjectPose('leftbottomcylinder', airsim.Pose(airsim.Vector3r(25, 30, 0)))
    right_bottom = client.simSetObjectPose('rightbottomcylinder',airsim.Pose(airsim.Vector3r(0, 30, 0)))
    left_top = client.simSetObjectPose('leftopcylinder',airsim.Pose(airsim.Vector3r(25, 0, 0)))
    right_top = client.simSetObjectPose('rightopcylinder',airsim.Pose(airsim.Vector3r(0, 0, 0)))


    chargingstation = client.simSetObjectPose('ChargingStation1',airsim.Pose(airsim.Vector3r(12.5, 36, 0)))
    chargingstation2 = client.simSetObjectPose('ChargingStation2',airsim.Pose(airsim.Vector3r(12.5, -6, 0)))
    belt = client.simSetObjectPose('M_Conveyor_BP_2', airsim.Pose(airsim.Vector3r(12.5,15,0)))

client = Client()
place_objects(client)

file = 'stag_rose_cropped'
dispatcher_loc = [[25, 30, 0], [0, 30, 0], [25, 0, 0], [0, 0, 0]]
dispatcher_loc = {k:v for k,v in enumerate(dispatcher_loc)}
quota_balanced, recompute = QuotaBalanced(np.load(f'{file}_points.npy'), dispatcher_loc)

#print(quota_balanced)

## source = {index : [25,30,0] }
## target = {index :  }
# sources = {}
# target = {}
# obstacles = set()
# for key,value in quota_balanced.items():
    
#     for val in value:
#         sources[val[0]] = dispatcher_loc[key]
#         target[val[0]] = val[1]
#         obstacles.add(val[0])
    
# print(sources)
# print(target)


max_drones = max(map(lambda x: len(quota_balanced[x]), quota_balanced))
count = 0
obstacles = set()

progress = tqdm.tqdm(total=sum(map(lambda x: len(quota_balanced[x]), quota_balanced)))
processes = []

scene = PointCloud().load_from_npy_file('./stag_rose_cropped')
path_planner = APFPathPlanner(client, similar_swaps_only=False)
orc = StagOrchestrator(client, scene, APFPathPlanner(client), delay=1, dispatch_order=quota_balanced, standalone=False)

# with multiprocessing.get_context('fork').Pool() as pool:
while count < max_drones:
    sources, targets = {}, {}
    for dispatcher in dispatcher_loc:
        if count < len(quota_balanced[dispatcher]):
            drone, target = quota_balanced[dispatcher][count]
            name = f'fls_{drone}'
            x, y, z = target
            targets[name] = np.array([x, y, -z])
            sources[name] = np.array(dispatcher_loc[dispatcher])
            obstacles |= {name}
            color = (0, 0, 0)
            client.simAddVehicle(name, airsim.Vector3r(*dispatcher_loc[dispatcher]), color)    
        progress.update()
    # print(ources, targets, obstacles, list(map(lambda x: (x, client.drones[x].position.to_numpy_array()), client.drones)))
    # sys.exit()
    path_planner.setup(sources, targets, obstacles)
    # pool.apply_async(path_planner.run_update_loop)
    # process = Process(target=path_planner.run_update_loop)
    # processes.append(process)
    # process.start()
    path_planner.run_update_loop(0.01)
    count += 1
        # time.sleep(2)
    # print(client.drones[drone]._path)

# for process in processes:
#     process.join()

for drone in client.drones:
    client.drones[drone].color = tuple(scene.colors[int(drone.split('_')[-1])]) 

time.sleep(5)

orc.run()











# a = [[[[None,None,None]] * 2 * idx + p for idx,p in enumerate(path[disp])] for disp in range(len(path))]
# # print(a[0][0])

# # max_length = [[max(len(item) for item in a[0])],[max(len(item) for item in a[1])]]
# max_length = [max(len(item) for item in a[i])for i in range(len(a))]

# a = [[ p + [[None,None,None]]* (max_length[disp] - len(p)) for idx,p in enumerate(a[disp])] for disp in range(len(a))]

# print(max_length)
# # print(path[0][3])
# print(a[0][3])
# max_length_1 = [len(item) == max_length[i]  for i in range(len(a)) for item in a[i]]
# print(all(max_length_1))

# for i in range(len(a)):
#     with open(f'dispatcher_{i}.npy', 'wb') as f:
#         np.save(f, np.array(a[i]))