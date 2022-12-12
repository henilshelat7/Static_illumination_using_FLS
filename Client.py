from typing import Tuple, Dict

import airsim

from Drone import Drone

class Client(airsim.MultirotorClient):
    def __init__(self, ip='', port=41451, timeout_value=3600):
        super().__init__(ip, port, timeout_value)
        self._drones: Dict[str, Drone] = {}
        self._vehicle_type = 'simpleflight'
        self._pawn_path = 'FLSDronePawn'
        self.confirmConnection()
        self.enableApiControl(True)
    
    @property
    def drones(self):
        return self._drones

    def _register_drone(self, name: str, position: airsim.Vector3r, charging) -> Drone:
        drone = Drone(name, position, self, charging)
        self._drones[name] = drone
        return drone

    def simAddVehicle(self, vehicle_name, position: airsim.Vector3r, color: Tuple[int], charging: bool=False):
        # print(vehicle_name, position, color)
        created = super().simAddVehicle(vehicle_name, self._vehicle_type, airsim.Pose(position), self._pawn_path)
        drone = self._register_drone(vehicle_name, position, charging)
        drone.color = color
        return created
