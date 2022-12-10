import numpy as np
from QuotaBalanced import QuotaBalanced
from move_path_loc import compute_path


locations = np.load('0_points.npy')
dispatcher_loc = [[25, 30, 0], [0, 30, 0], [25, 0, 0], [0, 0, 0]]
quota_balanced, recompute = QuotaBalanced(locations, dispatcher_loc)
j = 0
print(quota_balanced)
path = [[],[],[],[]]
for i in range(4):
    x = quota_balanced[i]
    
    location = dispatcher_loc[i]
    for k in range(len(x)):
        
        path[i].append(compute_path(location, locations[x[k]]))

# print(path[min_dist[0][0]], location[min_dist[0][0]])
print(locations[quota_balanced[0][3]])

a = [[[[None,None,None]] * 2 * idx + p for idx,p in enumerate(path[disp])] for disp in range(len(path))]
# print(a[0][0])

# max_length = [[max(len(item) for item in a[0])],[max(len(item) for item in a[1])]]
max_length = [max(len(item) for item in a[i])for i in range(len(a))]

a = [[ p + [[None,None,None]]* (max_length[disp] - len(p)) for idx,p in enumerate(a[disp])] for disp in range(len(a))]

print(max_length)
# print(path[0][3])
print(a[0][3])
max_length_1 = [len(item) == max_length[i]  for i in range(len(a)) for item in a[i]]
print(all(max_length_1))

for i in range(len(a)):
    with open(f'dispatcher_{i}.npy', 'wb') as f:
        np.save(f, np.array(a[i]))
