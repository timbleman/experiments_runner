from beamngpy import Vehicle, BeamNGpy, os
from beamngpy.sensors import Electrics

import numpy as np
from scipy import stats
from shapely.geometry import Polygon, LineString, Point

from prefab_parser import PrefabParser

from typing import Dict, List

NUM_BINS = 16
STEERING_RANGE = (-1, 1)
THROTTLE_BRAKE_RANGE = (0, 1)
SPEED_RANGE = (0, 100)


class CoverageCollector:
    def __init__(self, vehicle: Vehicle, bng: BeamNGpy, prefab_path):
        """ initializes CoverageCollector and adds electrics sensor to vehicle
            has to be called for each new vehicle

        :param vehicle: BeamNG vehicle
        :param bng: BeamNG instance
        """
        prefab_parser = PrefabParser(prefab_path)
        # maybe look if the desired road exists?
        # if self.roads is None:
        #    self.roads = self.bng.get_roads()  #self.bng.get_roads() #get_road_edges('main_road')
        #    print(self.roads)
        self.main_road = prefab_parser.parse_road('road_0')
        self.road_width = prefab_parser.get_road_width()

        # to check whether the car just left the road
        self.previously_offroad = True

        self.speed_arr = []
        self.throttle_arr = []
        self.steering_arr = []
        self.brake_arr = []
        self.distance_arr = []

        self.vehicle = vehicle
        self.bng = bng
        electrics = Electrics()
        self.vehicle.attach_sensor("electrics", electrics)

    def collect(self):
        """ Collects sensor data in specific lists, must be called at a fixed number of steps
            may need a Lock() like in DriveBuild

        :return: none
        """
        sensors = self.bng.poll_sensors(self.vehicle)
        electronics = sensors["electrics"]["values"]
        state = self.vehicle.state

        # speed in kph
        self.speed_arr.append(np.linalg.norm(state["vel"]) * 3.6)
        self.steering_arr.append(electronics["steering_input"])
        self.throttle_arr.append(electronics["throttle_input"])
        self.brake_arr.append(electronics["brake_input"])
        car_position = (self.vehicle.state['pos'][0], self.vehicle.state['pos'][1])
        #print("pos? ", car_position)
        distance_to_c = self._distance_to_center(car_position)
        self.distance_arr.append(distance_to_c)
        # look if the car is leaving the road
        if self._is_offroad(distance_to_c):
            print("offroad!")
            if not self.previously_offroad:
                self._obe_coverage(self.speed_arr[-1], state)
                self.previously_offroad = True
        else:
            self.previously_offroad = False

    def _obe_coverage(self, speed: float, state: Dict):
        """ Get only called when an OBE occurs, the coverage gets collected

        :param speed: the current speed of the vehicle
        :param state: state of vehicle as dict, includes speed, position and direction vectors
        :return: None
        """
        print("new obe at speed ", speed, " state ", state)

    def _distance_to_center(self, car_pos):
        return self.main_road.distance(Point(car_pos))

    def _is_offroad(self, distance_to_c: int):
        # TODO this simple factor may need some tweaking
        return distance_to_c > (self.road_width * 0.5)

    def get_bins(self, data: list, bounds):
        """ Returns a list of bins for an array, the bins are non-binary but counting

        :param data: List of data points
        :param bounds: Tuple from start to end (start, end), values outside get discard
        :return: List of bins
        """
        bins, bin_edges, binnum = stats.binned_statistic(data, data, bins=NUM_BINS, range=bounds, statistic='count')
        return bins

    def get_steering_bins(self):
        return self.get_bins(self.steering_arr, STEERING_RANGE)

    def get_throttle_bins(self):
        return self.get_bins(self.throttle_arr, THROTTLE_BRAKE_RANGE)

    def get_brake_bins(self):
        return self.get_bins(self.brake_arr, THROTTLE_BRAKE_RANGE)

    def get_speed_bins(self):
        # TODO change the bounds, find a good compromise, maybe dynamically
        return self.get_bins(self.speed_arr, SPEED_RANGE)

    def get_distance_bins(self, bounds):
        return self.get_bins(self.distance_arr, bounds)

    def get_speed_steering_2d(self):
        """ Returns a two dimensional array of bins with the steering input as x-axis and the speed as y-axis

        :return: histogram as a two-dimensional array
        """
        histogram, steering_edges, speed_edges = np.histogram2d(self.steering_arr, self.speed_arr, bins=NUM_BINS,
                                                                range=(STEERING_RANGE, SPEED_RANGE), normed=False)
        return histogram
