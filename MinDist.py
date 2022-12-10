from airsim import Vector3r
import collections
import math

def MinDist(poses, dispatchers):
    """Implementation of MinDist Algorithm in Shahram Ghandeharizadeh. 
        2022. Display of 3D Illuminations using Flying Light Specks

    Args:
        poses (list[list[int]]): positions of drone target points
        dispatchers (list[list[int]]): positions of drone dispathcer points

    Returns:
        dict[int, list[int]]: the deloyment of drones sorted by descending 
        distance for each dispatcher
    """    
    deploy = collections.defaultdict(list)
    for i in range(len(poses)):
        minDist = math.inf
        tgt = -1
        for j in range(len(dispatchers)):
            dist = Vector3r(*poses[i]).distance_to(Vector3r(*dispatchers[j]))
            if dist < minDist:
                minDist = dist
                tgt = j
        deploy[tgt].append(i)
    for dispather in deploy:
        deploy[dispather].sort(reverse=True, key = lambda v: Vector3r(*poses[v]).distance_to(Vector3r(*dispatchers[dispather])))
    return deploy
