import time
from copy import deepcopy

import numpy as np
import airsim
from Client import Client
from DroneAnalyzer import DroneAnalyzer

class APFPlannerParams:
    k_attraction = 700
    d_repulsion_in = 1
    d_repulsion_out = 10
    d_repulsion_bound = 15
    k_repulsion_in = 300
    k_repulsion_out = 50
    k_repulsion_bound = 500
    power_factor = 3
    step_length = 2
    d_threshold = 2
    d_obstacle_threshold = 0.0001

class APFPathPlanner():
    def __init__(self, client: Client, similar_swaps_only: bool=True):
        self._client = client
        self._params = APFPlannerParams()
        self._no_change_threshold = 50
        self._interception_steps = 15
        self._teleport_threshold = 1500
        self._similar_swaps = similar_swaps_only
        self._analyzer = DroneAnalyzer(self._client)
        self._bounds = None

    def set_bounds(self, bounds):
        self._bounds = {
            'z_range_min': 0, 'z_range_max': bounds['centers'][0][2] * 2,
            'x_min': bounds['maxs'][0][0] + self._params.d_repulsion_bound,
            'x_max': bounds['maxs'][1][0] - self._params.d_repulsion_bound,
            'y_min': bounds['maxs'][2][1] + self._params.d_repulsion_bound,
            'y_max': bounds['maxs'][3][1] - self._params.d_repulsion_bound,
            'y_range_min': bounds['maxs'][1][1], 'y_range_max': bounds['mins'][1][1],
            'x_range_min': bounds['maxs'][3][0], 'x_range_max': bounds['mins'][3][0]
        }
    
    def _get_boundary_repulsive_force(self, source):
        if source[0] < self._bounds['x_min'] or source[0] > self._bounds['x_max']:
            return np.array([
                -1 * np.sign(source[0]) * self._params.k_repulsion_bound * min(abs(self._bounds['x_min'] - source[0]), abs(self._bounds['x_max'] - source[0])),
                np.clip(source[1] + np.random.randint(-10, 10), self._bounds['y_range_min'], self._bounds['y_range_max']),
                np.clip(source[2] + np.random.randint(-10, 10), self._bounds['z_range_min'], self._bounds['z_range_max'])
            ])
        elif source[1] < self._bounds['y_min'] or source[1] > self._bounds['y_max']:
            return np.array([
                np.clip(source[0] + np.random.randint(-10, 10), self._bounds['x_range_min'], self._bounds['x_range_max']),
                -1 * np.sign(source[1]) * self._params.k_repulsion_bound * min(abs(self._bounds['y_min'] - source[1]), abs(self._bounds['y_max'] - source[1])),
                np.clip(source[2] + np.random.randint(-10, 10), self._bounds['z_range_min'], self._bounds['z_range_max'])
            ])
        else: return np.zeros((3,))

    def _get_repulsive_force(self, source, target, obstacle):
        obstacle_dist, target_dist = np.clip(np.linalg.norm(source - obstacle), self._params.d_obstacle_threshold, np.inf), np.linalg.norm(source - target)
        if obstacle_dist <= self._params.d_repulsion_in:
            coeff = self._params.k_repulsion_in
            bound = self._params.d_repulsion_in
        elif obstacle_dist <= self._params.d_repulsion_out and obstacle_dist > self._params.d_repulsion_in:
            coeff = self._params.k_repulsion_out
            bound = self._params.d_repulsion_out
        else: return np.zeros((3,))
        force_1 = -coeff * ((1 / obstacle_dist) - (1 / bound)) * (1 / (obstacle_dist ** 2)) * ((source - target) ** self._params.power_factor) * ((source - obstacle) / obstacle_dist)
        force_2 = (-self._params.power_factor / 2) * coeff * (((1 / obstacle_dist) - (1 / bound)) ** 2) * ((source - target) ** (self._params.power_factor - 1)) * ((source - target) / target_dist)
        return force_1 + force_2

    def _get_attractive_force(self, source, target):
        return self._params.k_attraction * (target - source)
    
    def _get_artificial_force(self, source, target, obstacles):
        attractive_force = self._get_attractive_force(source, target)
        repulsive_force = np.sum(np.array([self._get_repulsive_force(source, target, obstacle).tolist() for obstacle in obstacles]), 0)
        if self._bounds:
            boundary_force = self._get_boundary_repulsive_force(source)
            if any(boundary_force): return boundary_force
        return attractive_force + repulsive_force
    
    def _compute_outward_vector(self, drone):
        outward_vector = deepcopy(self._client.drones[drone].position).to_numpy_array() - self._center_of_illumination
        return  outward_vector / np.linalg.norm(outward_vector)

    def _send_drones_near_center_outwards(self, drones):
        outward_vectors = {}
        for drone in drones:
            outward_vectors[drone] = self._compute_outward_vector(drone)
        sorted_drones = sorted([(drone, self._analyzer.compute_distance(self._client.drones[drone].position.to_numpy_array(), self._center_of_illumination)) for drone in drones], key=lambda x: x[1], reverse=True)
        for _ in range(self._interception_steps):
            for drone, _ in sorted_drones:
                self._client.drones[drone].position = deepcopy(self._client.drones[drone].position) + airsim.Vector3r(*outward_vectors[drone])
        
    def _step(self):
        for drone in self._sources:
            if drone not in self._reached:
                obstacles = [deepcopy(self._obstacles[obstacle]) for obstacle in self._obstacles if obstacle != drone]
                obstacles.extend([deepcopy(self._client.drones[obstacle].position).to_numpy_array() for obstacle in self._targets if obstacle != drone])
                current_position = deepcopy(self._client.drones[drone].position).to_numpy_array()
                artificial_force = self._get_artificial_force(current_position, deepcopy(self._targets[drone]), obstacles)
                next_position = airsim.Vector3r(*(current_position + ((artificial_force / np.linalg.norm(artificial_force)) * self._params.step_length)))
                if any(np.isnan(next_position.to_numpy_array())):
                    print(drone)
                else:
                    self._client.drones[drone].position = next_position
                if np.linalg.norm(deepcopy(next_position).to_numpy_array() - deepcopy(self._targets[drone])) < self._params.d_threshold:
                    self._client.drones[drone].position = airsim.Vector3r(*self._targets[drone])
                    self._reached |= {drone}
        self._target_exchange()

    def _target_exchange(self):
        for drone in self._sources:
            if drone not in self._reached:
                current_position = deepcopy(self._client.drones[drone].position).to_numpy_array()
                current_target = deepcopy(self._targets[drone])
                count, doi = 0, set()
                for reached_drone in self._reached:
                    repulsive_force = self._get_repulsive_force(current_position, current_target, deepcopy(self._client.drones[reached_drone].position).to_numpy_array())
                    if any(repulsive_force):
                        if self._similar_swaps:
                            if self._client.drones[reached_drone].is_charging() == self._client.drones[drone].is_charging():
                                count += 1
                                doi |= {reached_drone}
                        else:
                            count += 1
                            doi |= {reached_drone}
                if count >= 2 and np.linalg.norm(current_position - current_target) > np.linalg.norm(deepcopy(self._client.drones[drone].previous_position).to_numpy_array() - current_target):
                    to_exchange = None
                    min_dist = np.inf
                    for possible_exchange in doi:
                        if np.linalg.norm(deepcopy(self._client.drones[possible_exchange].position).to_numpy_array() - current_target) < np.linalg.norm(current_position - current_target):
                            dist = np.linalg.norm(current_position - deepcopy(self._client.drones[possible_exchange].position).to_numpy_array())
                            if dist < min_dist:
                                to_exchange = possible_exchange
                                min_dist = dist
                    if to_exchange:
                        self._targets[drone] = deepcopy(self._targets[to_exchange])
                        self._targets[to_exchange] = deepcopy(current_target)
                        self._reached -= {to_exchange}
    
    def _teleport(self, drones):
        for drone in drones:
            self._client.drones[drone].position = airsim.Vector3r(*self._targets[drone])
            self._reached |= {drone}

    def setup(self, sources: dict, targets: dict, obstacles):
        self._sources = sources
        self._targets = targets
        self._obstacles = {obstacle: deepcopy(self._client.drones[obstacle].position).to_numpy_array() for obstacle in obstacles}
        self._reached = set()
        self._center_of_illumination = self._analyzer.compute_center()

    def run_update_loop(self, delay=0, debug=False):
        step_counter, no_change_counter, prev_num_reached = 0, 0, 0
        while len(self._reached) != len(self._sources.keys()):
            start = time.time()
            self._step()
            if step_counter == self._teleport_threshold:
                remaining_drones = set(deepcopy(self._sources).keys()) - deepcopy(self._reached)
                self._teleport(remaining_drones)
            if len(self._reached) == prev_num_reached:
                no_change_counter += 1
            else:
                prev_num_reached = len(self._reached)
                no_change_counter = 0
            if no_change_counter == self._no_change_threshold - 1:
                remaining_drones = set(deepcopy(self._sources).keys()) - deepcopy(self._reached)
                drones_close_to_source = self._analyzer.find_drones_very_close_to_source(remaining_drones, deepcopy(self._sources))
                # print(drones_close_to_source)
                self._send_drones_near_center_outwards((drones_close_to_source[True] | drones_close_to_source[False]))
                remaining_drones -= (drones_close_to_source[True] | drones_close_to_source[False])
                drones_flying_away = self._analyzer.find_drones_flying_away_from_target(remaining_drones, deepcopy(self._targets))
                # print(drones_flying_away)
                remaining_drones -= (drones_flying_away[True] | drones_flying_away[False])
                # print(remaining_drones)
                # for drone in remaining_drones:
                #     print(drone, ':', self._client.drones[drone].position.to_numpy_array(), self._targets[drone])
                prev_num_reached = len(self._reached)
                no_change_counter = 0
                # break
            stop = time.time()
            if debug: print('Step #:', step_counter, '| # Reached:', len(self._reached))
            # print(stop-start)
            if delay > stop-start:
                time.sleep(delay-stop+start)
            step_counter += 1