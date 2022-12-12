import time
import math
from copy import deepcopy

import airsim


from Client import Client
from PointCloud import PointCloud
from APF import APFPathPlanner

class StagOrchestrator():
    FLIGHT_TIME = 30
    CHARGE_TIME = 10
    STATION_TOP_OFFSET = airsim.Vector3r(0, 0, -4.6)
    STATION_BOUND_X = 20
    STATION_BOUND_Y = 2
    STATION_GRID_X_INTERVAL = 1
    STATION_GRID_Y_INTERVAL = 2
    # STATIONS = ['GroupActor_2', 'GroupActor_3', 'GroupActor_1', 'GroupActor_0']
    STATIONS = ['ChargingStation1', 'ChargingStation2']
    BOUNDARY_CENTERS = ['Cube2', 'Cube3_1', 'Cube13', 'Cube14']
    BOUNDARY_MAXS = ['Sphere_ymax_Cube2', 'Sphere_ymax_Cube3_1', 'Sphere_xmax_Cube13', 'Sphere_xmax_Cube14']
    BOUNDARY_MINS = ['Sphere_ymin_Cube2', 'Sphere_ymin_Cube3_1', 'Sphere_xmin_Cube13', 'Sphere_xmin_Cube14']

    def __init__(self, client: Client, scene: PointCloud, planner: APFPathPlanner, standalone: bool=True, keep_color: bool=False, beta: int=FLIGHT_TIME, omega: int=CHARGE_TIME, delay: float=0, dispatch_order: dict = {}) -> None:
        super().__init__()
        self._client = client
        self._scene = scene
        self._delay = delay
        self._beta = beta
        self._omega = omega
        self._stag_interval = beta / scene.points.shape[0]
        self._grid = []
        self._charging = None
        self._illuminating = None
        self._charge_color = (150, 150, 150)
        self._sort = lambda x: sorted(x, key=lambda y: int(y.split('_')[1]))
        self._planner = planner
        self._standalone = standalone
        self._keep_color = keep_color
        self._dispatch_order = dispatch_order

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
        return math.ceil((self._get_next_id() * self._omega) / self._beta)
    
    def _add_extra_drones(self):
        stations = self._get_station_locations()
        num_extra_drones = self._compute_extra_drones()
        next_id = self._get_next_id()
        curr_positions = [station - airsim.Vector3r(self.STATION_BOUND_X, self.STATION_BOUND_Y, 0) for station in stations]
        i, station = 0, 0
        while i < num_extra_drones:
            name = f'fls_{next_id + i}'
            self._client.simAddVehicle(name, deepcopy(curr_positions[station]), self._charge_color, True)
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

    def _toggle_charging_status(self, drones):
        for drone in drones:
            self._client.drones[drone].toggle_charging_status()
            if self._client.drones[drone].is_charging() and not self._keep_color:
                self._client.drones[drone].color = self._charge_color

    def _change_color(self, colors):
        for drone in colors:
            self._client.drones[drone].color = colors[drone]

    def _run_stag_exchange_loop(self, idx, debug=False):
        sources, targets, colors = {}, {}, {}
        obstacles = set(self._illuminating)
        for i in range(len(self._charging)):
            start_position = deepcopy(self._client.drones[self._illuminating[idx]].position)
            end_position = deepcopy(self._client.drones[self._charging[i]].position)
            sources[self._illuminating[idx]] = start_position.to_numpy_array()
            targets[self._illuminating[idx]] = end_position.to_numpy_array()
            sources[self._charging[i]] = end_position.to_numpy_array()
            targets[self._charging[i]] = start_position.to_numpy_array()
            colors[self._charging[i]] = deepcopy(self._client.drones[self._illuminating[idx]].color)
            if debug: print(f'Pre-Exchange: {self._illuminating[idx]} {start_position.to_numpy_array().tolist()} {end_position.to_numpy_array().tolist()} {self._charging[i]}')
            self._charging[i], self._illuminating[idx] = self._illuminating[idx], self._charging[i]
            if debug: print(f'Post-Exchange: {self._illuminating[idx]} {self._client.drones[self._illuminating[idx]].position.to_numpy_array().tolist()} {self._client.drones[self._charging[i]].position.to_numpy_array().tolist()} {self._charging[i]}')
            idx += 1
            if idx == len(self._illuminating): idx = 0
        return idx, sources, targets, obstacles, colors

    def run(self, bounds: bool=False):
        if self._standalone:
            self._render_static_scene()
        self._illuminating = set(self._client.drones.keys())
        self._add_extra_drones()
        self._charging = set(self._client.drones.keys()) - self._illuminating
        if not self._dispatch_order: self._illuminating, self._charging = self._sort(list(self._illuminating)), self._sort(list(self._charging))
        else:
            count = 0
            max_drones = max(map(lambda x: len(self._dispatch_order[x]), self._dispatch_order))
            self._illuminating = []
            while count < max_drones:
                sources, targets = {}, {}
                for dispatcher in self._dispatch_order:
                    if count < len(self._dispatch_order[dispatcher]):
                        drone, target = self._dispatch_order[dispatcher][count]
                        self._illuminating.append(f'fls_{drone}')
                count += 1
        
            self._charging = self._sort(list(self._charging))            
                        
                 
        original = set(self._illuminating)
        idx = 0
        if not bounds: self._planner.set_bounds(self._get_environment_bounds())
        change_counter = 0
        while True:
            idx, sources, targets, obstacles, colors = self._run_stag_exchange_loop(idx)
            self._planner.setup(sources, targets, obstacles)
            self._planner.run_update_loop()
            self._toggle_charging_status(sources)
            if not self._keep_color: self._change_color(colors)
            change_counter += 1
            print(f'Loop {change_counter} completed...')
            # break
            time.sleep((self._stag_interval * 10))
            if set(self._illuminating) == original and self._standalone:
                print('Done...')    
                break
        