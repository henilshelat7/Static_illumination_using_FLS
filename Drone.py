from copy import deepcopy
from typing import Tuple

import airsim

class Drone:
    def __init__(self, name: str, position: airsim.Vector3r, client: airsim.MultirotorClient, charging: bool=False) -> None:
        self._name = name
        self._home_position = position
        self._color = (0, 0, 0)
        self._curr_position = position
        self._prev_position = None
        self._client = client
        self._color_command = 'ke ' + self._name + ' SetLightColor {r} {g} {b}'
        self._is_charging = charging
        self.ignore_collisions = True
        self._path = []

    @property
    def position(self):
        return self._curr_position
    
    @property
    def color(self):
        return self._color
    
    @property
    def name(self):
        return self._name
    
    @property
    def home(self):
        return self._home_position
    
    @property
    def previous_position(self):
        return self._prev_position

    @position.setter
    def position(self, pos: airsim.Vector3r):
        self._prev_position = self._curr_position
        self._curr_position = pos
        self._path.append(self._curr_position)
        self._client.simSetVehiclePose(airsim.Pose(self._curr_position - self._home_position), self.ignore_collisions, self._name)
    
    @color.setter
    def color(self, col: Tuple[int]):
        self._color = col
        r, g, b = self._color
        self._client.simRunConsoleCommand(self._color_command.format(r=r, g=g, b=b))

    def is_charging(self):
        return self._is_charging

    def toggle_charging_status(self):
        self._is_charging = not self._is_charging