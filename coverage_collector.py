from beamngpy import Vehicle, BeamNGpy, os
from beamngpy.sensors import Electrics

import numpy as np
from scipy import stats
from shapely.geometry import Polygon

# maybe impact information gathering, see DriveBuild
from threading import Lock

NUM_BINS = 16
STEERING_RANGE = (-1, 1)
THROTTLE_BRAKE_RANGE = (0, 1)
SPEED_RANGE = (0, 100)


class CoverageCollector:
    def __init__(self, vehicle: Vehicle, bng: BeamNGpy):
        """ initializes CoverageCollector and adds electrics sensor to vehicle
            has to be called for each new vehicle

        :param vehicle: BeamNG vehicle
        :param bng: BeamNG instance
        """
        self.speed_arr = []
        self.throttle_arr = []
        self.steering_arr = []
        self.brake_arr = []

        self.vehicle = vehicle
        self.bng = bng
        self.roads = None
        electrics = Electrics()
        self.vehicle.attach_sensor("electrics", electrics)

        self._sim_lock = Lock()

    def collect(self):
        """ Collects sensor data in specific lists, must be called at a fixed number of steps

        :return: none
        """
        if self.roads is None:
            self.roads = self.bng.get_roads()  #self.bng.get_roads() #get_road_edges('main_road')
            print(self.roads)

            #copied from DriveBuild
            self._sim_lock.acquire()
            self.roads = self.bng.get_road_edges('main_road')
            self._sim_lock.release()
            # print("self.bng.scenario-_get_roads_list: ", self.bng.scenario.get_roads_list())

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
        #self._is_offroad()

    def _is_offroad(self):
        from typing import Dict, List
        #bbox = self._poll_request_data()[0]
        # TODO this blocks everything, needs second socket?
        #bbox = self.vehicle.get_bbox()
        #bbox = self.bng.get_vehicle_bbox(self.vehicle)
        bbox = True
        #print("hi0, bbox: ", bbox)

        def _to_polygon(road_edges: List[Dict[str, float]]) -> Polygon:
            points = [p["left"][0:2] for p in road_edges]
            right_edge_points = [p["right"][0:2] for p in road_edges]
            right_edge_points.reverse()
            points.extend(right_edge_points)
            return Polygon(shell=points)

        if bbox:
            offroad = True
            #print("self.bng.scenario.roads: ", self.bng.scenario.roads)
            for road in self.roads:
                print("road.rid: ", road.rid)
                edges = self.bng.get_road_edges(road.rid)
                polygon = _to_polygon(edges)
                print("hi3")
                '''
                if polygon.intersects(bbox):
                    offroad = False
                    print("hi4")
                    break
                '''
            return offroad
        else:
            raise Exception('Could not get bbox')

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

    def get_speed_steering_2d(self):
        """ Returns a two dimensional array of bins with the steering input as x-axis and the speed as y-axis

        :return: histogram as a two-dimensional array
        """
        histogram, steering_edges, speed_edges = np.histogram2d(self.steering_arr, self.speed_arr, bins=NUM_BINS,
                                                                range=(STEERING_RANGE, SPEED_RANGE), normed=False)
        return histogram
