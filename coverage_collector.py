from beamngpy import Vehicle, BeamNGpy, os
from beamngpy.sensors import Electrics

import numpy as np
from scipy import stats
from shapely.geometry import Polygon, LineString, Point, MultiPoint, mapping
from shapely import ops

from prefab_parser import PrefabParser

from typing import Dict, List

NUM_BINS = 16
STEERING_RANGE = (-1, 1)
THROTTLE_BRAKE_RANGE = (0, 1)
SPEED_RANGE = (0, 100)
ANGLE_RANGE = (-np.pi, np.pi)


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
        self.obe_speed_arr = []
        self.obe_angle_arr = []

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
        def angle_between_3_points(point_road: Point, point_intersect: Point, point_car: Point):
            a = (point_intersect.x - point_road.x, point_intersect.y - point_road.y)
            b = (point_intersect.x - point_car.x, point_intersect.y - point_car.y)
            numerator = np.multiply(a, b)
            # TODO wrong calculation
            angle = np.arctan2(point_road.y - point_intersect.y, point_road.x - point_intersect.x) - \
                                np.arctan2(point_car.y - point_intersect.y, point_car.x - point_intersect.x)

            return angle

        car_position = (state['pos'][0], state['pos'][1])
        car_velocity = (state["vel"][0], state["vel"][1])
        assert car_velocity is not (0, 0), "An OBE cannot happen at 0 kph"
        factor = 5
        #print("Car position and volcity and volocity multiplied: ", car_position, car_velocity, np.multiply(factor, car_velocity))
        car_vector = [0, car_position, 0]
        intersecting = False
        # increase the length of the vector if the lines do not intersect
        while(not intersecting):
            car_vector[0] = np.subtract(car_position, np.multiply(factor, car_velocity))
            car_vector[2] = np.add(car_position, np.multiply(factor, car_velocity))
            car_vector_lstr = LineString(car_vector)
            intersecting = self.main_road.intersects(car_vector_lstr)
            factor += 5

        car_vector_lstr = LineString(car_vector)
        intersecting_point = self.main_road.intersection(car_vector_lstr)
        print("intersection point: ", intersecting_point)

        # turn road to multipoint series to find nearest neighbor
        # TODO find a better way to get the closest point
        main_road_multipoint = MultiPoint(np.array(self.main_road))
        orig_point, nearest_point = ops.nearest_points(intersecting_point, main_road_multipoint)
        # print("main_road_multipoint ", main_road_multipoint)
        # print("orig_point, nearest_point ", orig_point, nearest_point)

        obe_angle = angle_between_3_points(nearest_point, intersecting_point, Point(car_position))
        print("angle ", obe_angle)

        self.obe_speed_arr.append(np.linalg.norm(state["vel"]) * 3.6)
        self.obe_angle_arr.append(obe_angle)

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

    def get_obe_speed_angle_bins(self):
        histogram, speed_edges, angle_edges = np.histogram2d(self.obe_speed_arr, self.obe_angle_arr, bins=NUM_BINS,
                                                                range=(SPEED_RANGE, ANGLE_RANGE), normed=False)
        return histogram
