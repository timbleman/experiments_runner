from beamngpy import Vehicle, BeamNGpy, os
from beamngpy.sensors import Electrics

import numpy as np
from scipy import stats

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
        electrics = Electrics()
        self.vehicle.attach_sensor("electrics", electrics)

    def collect(self):
        """ Collects sensor data in specific lists, must be called at a fixed number of steps

        :return: none
        """
        sensors = self.bng.poll_sensors(self.vehicle)
        electronics = sensors["electrics"]["values"]
        state = self.vehicle.state

        # TODO speed not correct
        self.speed_arr.append(np.linalg.norm(state["vel"]))
        self.steering_arr.append(electronics["steering_input"])
        self.throttle_arr.append(electronics["throttle_input"])
        self.brake_arr.append(electronics["brake_input"])

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
