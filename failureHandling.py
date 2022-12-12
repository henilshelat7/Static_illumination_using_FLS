import time
import random
import math
from copy import deepcopy

import airsim

from Client import Client
from PointCloud import PointCloud
from APF import APFPathPlanner


class FailureOrchestrator():
    FAILURE_PROBABILITY = 0.2
    NUM_FAILURES_TO_SIMULATE = 5
    MAX_NUM_DRONES_TO_FAIL = 5
    STATION_TOP_OFFSET = airsim.Vector3r(0, 0, -4.6)
    GRAVITY_VECTOR = airsim.Vector3r(0, 0, 0.5)
    STATION_BOUND_X = 20
    STATION_BOUND_Y = 2
    STATION_GRID_X_INTERVAL = 1
    STATION_GRID_Y_INTERVAL = 2
    # STATIONS = ['GroupActor_2', 'GroupActor_3', 'GroupActor_1', 'GroupActor_0']
    STATIONS = ['ChargingStation1', 'ChargingStation2']
    BOUNDARY_CENTERS = ['Cube2', 'Cube3_1', 'Cube13', 'Cube14']
    BOUNDARY_MAXS = ['Sphere_ymax_Cube2', 'Sphere_ymax_Cube3_1', 'Sphere_xmax_Cube13', 'Sphere_xmax_Cube14']
    BOUNDARY_MINS = ['Sphere_ymin_Cube2', 'Sphere_ymin_Cube3_1', 'Sphere_xmin_Cube13', 'Sphere_xmin_Cube14']

    def __init__(self, client: Client, scene: PointCloud, planner: APFPathPlanner, standalone: bool=True, keep_color: bool=False, p: float=FAILURE_PROBABILITY, delay: float=0) -> None:
        super().__init__()
        self._client = client
        self._scene = scene
        self._delay = delay
        self._p = p
        self._grid = []
        self._backup_map = {}
        self._backup = None
        self._illuminating = None
        self._standby_color = (0, 0, 0)
        self._sort = lambda x: sorted(x, key=lambda y: int(y.split('_')[1]))
        self._planner = planner
        self._standalone = standalone
        self._keep_color = keep_color

    def _get_next_id(self):
        return len(self._client.drones.keys())

    def _get_station_locations(self):
        return [self._client.simGetObjectPose(station).position + self.STATION_TOP_OFFSET for station in self.STATIONS]

    def _get_environment_bounds(self):
        return {
            'centers': [self._client.simGetObjectPose(bound).position.to_numpy_array() for bound in self.BOUNDARY_CENTERS], 
            'maxs': [self._client.simGetObjectPose(bound).position.to_numpy_array() for bound in self.BOUNDARY_MAXS], 
            'mins': [self._client.simGetObjectPose(bound).position.to_numpy_array() for bound in self.BOUNDARY_MINS]
        }

    def _render_static_scene(self):
        for idx, (point, color) in enumerate(zip(self._scene.points, self._scene.colors)):
            name = f'fls_{idx}'
            y, x, z = point
            self._client.simAddVehicle(name, airsim.Vector3r(x, y, -z), tuple(color))
        time.sleep(self._delay)

    def _compute_extra_drones(self) -> int:
        return self._get_next_id()
    
    def _add_extra_drones(self):
        
        stations = self._get_station_locations()
        num_extra_drones = self._compute_extra_drones()
        next_id = self._get_next_id()
        curr_positions = [station - airsim.Vector3r(self.STATION_BOUND_X, self.STATION_BOUND_Y, 0) for station in stations]
        i, station = 0, 0
        while i < num_extra_drones:
            self._backup_map[f'fls_{next_id + i}'] = f'fls_{i}'
            self._backup_map[f'fls_{i}'] = f'fls_{next_id + i}'
            name = f'fls_{next_id + i}'
            self._client.simAddVehicle(name, deepcopy(curr_positions[station]), self._standby_color, True)
            self._grid.append(deepcopy(curr_positions[station]))
            i += 1
            if (curr_positions[station].x_val + self.STATION_GRID_X_INTERVAL) > (stations[station].x_val + self.STATION_BOUND_X):
                curr_positions[station].x_val = (stations[station].x_val - self.STATION_BOUND_X)
                curr_positions[station].y_val += self.STATION_GRID_Y_INTERVAL
            else:
                curr_positions[station].x_val += self.STATION_GRID_X_INTERVAL
            station += 1
            if station == len(stations):
                station = 0
        time.sleep(self._delay)

    def _detect_failures(self, points_of_failures):
      
        failures = {point_of_failure: deepcopy(self._client.drones[point_of_failure].position).to_numpy_array() for point_of_failure in points_of_failures}
        
        colors = {self._backup_map[point_of_failure]: deepcopy(self._client.drones[point_of_failure].color) for point_of_failure in points_of_failures}
        reached = set()
        for point_of_failure in points_of_failures:
            print(f'Drone {point_of_failure} failing...')
        collision_order = [point_of_failure for point_of_failure in points_of_failures]
        while len(reached) < len(list(failures.keys())):
            collided = []
            for drone in failures:
                self._client.drones[drone].color = self._standby_color
                current_position = deepcopy(self._client.drones[drone].position)
                # print(drone, current_position.x_val, current_position.y_val, current_position.z_val)
                self._client.drones[drone].ignore_collisions = False
                if current_position.z_val < -0.5:
                    self._client.drones[drone].position = current_position + self.GRAVITY_VECTOR
                else:
                    current_position.z_val = -0.3
                    self._client.drones[drone].position = current_position
                    reached |= {drone}
                collision_info = self._client.simGetCollisionInfo(drone)
                if collision_info.has_collided and collision_info.object_name not in failures and collision_info.object_name in self._client.drones:
                    print(f'Drone {collision_info.object_name} failing because of collision...')
                    collided.append(collision_info.object_name)
                    collision_order.append(collision_info.object_name)
            for drone in collided:
                failures[drone] = deepcopy(self._client.drones[drone].position).to_numpy_array()
                colors[self._backup_map[drone]] = deepcopy(self._client.drones[drone].color)
        for drone in failures:
            self._client.drones[drone].ignore_collisions = True
            self._client.drones[drone].position = airsim.Vector3r(*deepcopy(failures[drone]))    
        
        return list(reversed(collision_order)), failures, colors

    def _make_failing_drones_fall(self, failures):
        reached = set()
        while len(reached) < len(failures):
            for drone in failures:
                current_position = deepcopy(self._client.drones[drone].position)
                # print(drone, current_position.x_val, current_position.y_val, current_position.z_val)
                if current_position.z_val < -0.5:
                    self._client.drones[drone].position = current_position + self.GRAVITY_VECTOR
                else:
                    current_position.z_val = -0.3
                    self._client.drones[drone].position = current_position
                    reached |= {drone}
            time.sleep(self._delay/5)

    def _change_color(self, colors):
        for drone in colors:
            self._client.drones[drone].color = colors[drone]

    def _run_failure_handling(self):
        num_drones_to_fail = random.randint(1, self.MAX_NUM_DRONES_TO_FAIL)
        points_of_failures = random.choices(self._illuminating, k=num_drones_to_fail)
        # points_of_failures = ['fls_37']
        collisions, failures, colors = self._detect_failures(points_of_failures)
        self._make_failing_drones_fall(collisions)
        replace_sources = {self._backup_map[failed]: deepcopy(self._client.drones[self._backup_map[failed]].position).to_numpy_array() for failed in failures}
        replace_targets = {self._backup_map[failed]: failures[failed] for failed in failures}
        self._change_color(colors)
        self._planner.setup(replace_sources, replace_targets, deepcopy(self._illuminating))
        self._planner.run_update_loop(delay=self._delay/5)
        # print(collisions, colors)
        time.sleep(self._delay)
        failed_sources = {failed: deepcopy(self._client.drones[failed].position).to_numpy_array() for failed in failures}
        failed_targets = {self._backup_map[replace_source]: replace_sources[replace_source] for replace_source in replace_sources}
        self._planner.setup(failed_sources, failed_targets, deepcopy(self._illuminating))
        self._planner.run_update_loop(delay=self._delay/5)
        for drone in failed_sources:
            self._illuminating.remove(drone)
            self._backup.append(drone)
        for drone in replace_sources:
            self._backup.remove(drone)
            self._illuminating.append(drone)

    def _run_failure_loop(self):
        if random.random() < self._p:
            print('Failure Occured...')
            self._run_failure_handling()
            return
        else:
            time.sleep(self._delay)
        self._run_failure_loop()

    def run(self):
        if self._standalone:
            self._render_static_scene()
        self._illuminating = set(self._client.drones.keys())
        self._add_extra_drones()
        self._backup = set(self._client.drones.keys()) - self._illuminating
        self._illuminating, self._backup = self._sort(list(self._illuminating)), self._sort(list(self._backup))
        self._planner.set_bounds(self._get_environment_bounds())
        for _ in range(self.NUM_FAILURES_TO_SIMULATE):
            self._run_failure_loop()