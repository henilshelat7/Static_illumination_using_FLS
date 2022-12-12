import math
from re import T
import collections
import numpy as np
from airsim import Vector3r

def QuotaBalanced(poses, dispatchers, rate=4, speed=10):
    """Implementation of MinDist Algorithm in Shahram Ghandeharizadeh. 
        2022. Display of 3D Illuminations using Flying Light Specks

    Args:
        poses (list[list[int]]): positions of drone target points
        dispatchers (list[list[int]]): positions of drone dispathcer points

    Returns:
        dict[int, list[int]]: the deloyment of drones sorted by descending 
        distance for each dispatcher
    """    
    quotas = {i: len(poses) / (len(dispatchers) * rate) for i in range(len(dispatchers))}
    active = {i for i in quotas.keys()}
    recompute = 0
    deploy = collections.defaultdict(list)
    for i in range(len(poses)):
        minDist = math.inf
        tgt = -1
        for j in active:
            dist = Vector3r(*poses[i]).distance_to(Vector3r(*dispatchers[j]))
            if dist < minDist:
                minDist = dist
                tgt = j
        deploy[tgt].append((i,np.array(poses[i])))
        
        t = minDist / speed
        quotas[tgt] -= t
        if quotas[tgt] <= 0:
            active -= {tgt}
        if len(active) == 0:
            recompute += 1
            quotas = {idx: (len(poses) - i) / (len(dispatchers) * rate) for idx in range(len(dispatchers)) if ((len(poses) - i) / (len(dispatchers) * rate)) > 0}
            active = {i for i in quotas.keys()}
    for dispather in deploy:
        deploy[dispather].sort(reverse=True, key = lambda v: Vector3r(*v[1]).distance_to(Vector3r(*dispatchers[dispather])))
    return deploy, recompute
