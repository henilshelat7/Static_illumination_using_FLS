from copy import deepcopy
from typing import Dict

import numpy as np

from Client import Client

class DroneAnalyzer:
    def __init__(self, client: Client) -> None:
        self._client = client
        self._closeness_threshold = 50

    def compute_center(self):
        illuminating = [deepcopy(drone.home.to_numpy_array()) for name, drone in self._client.drones.items()]
        return sum(illuminating) / len(illuminating)

    def compute_distance(self, a, b):
        return np.linalg.norm(deepcopy(a) - deepcopy(b))

    def _is_distance_in_bound(self, drone, source, bound) -> bool:
        return self.compute_distance(self._client.drones[drone].position.to_numpy_array(), source) < bound
    
    def _is_moving_away_from_target(self, drone, target) -> bool:
        return self.compute_distance(self._client.drones[drone].position.to_numpy_array(), target) > self.compute_distance(self._client.drones[drone].previous_position.to_numpy_array(), target)

    def find_drones_flying_away_from_target(self, drones, targets) -> Dict[bool, set]:
        drones_flying_away = {True: set(), False: set()}
        for drone in drones:
            if self._is_moving_away_from_target(drone, targets[drone]):
                drones_flying_away[self._client.drones[drone].is_charging()] |= {drone}
        return drones_flying_away
    
    def find_drones_very_close_to_source(self, drones, sources) -> Dict[bool, set]:
        drones_close_to_source = {True: set(), False: set()}
        for drone in drones:
            if self._is_distance_in_bound(drone, sources[drone], self._closeness_threshold):
                drones_close_to_source[self._client.drones[drone].is_charging()] |= {drone}
        return drones_close_to_source