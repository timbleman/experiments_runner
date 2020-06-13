from typing import Dict
import numpy as np

import coverage_collector
import utils

NUM_BINS = coverage_collector.NUM_BINS


class SuiteBehaviourComputer:
    def __init__(self, t_dict: Dict, start: int, end: int):
        self.test_dict = t_dict
        self.start = start
        self.end = end

    def calculate_suite_speed_steering_coverage(self):
        global_speed_steering_cov = np.zeros((NUM_BINS, NUM_BINS))
        for i in range(self.start, self.end + 1):
            cov_col = self.test_dict.get(str(i)).get('cov_collector')
            global_speed_steering_cov = np.add(global_speed_steering_cov, cov_col.get_speed_steering_2d())
        self.test_dict['whole_suite_speed_steering_coverage'] = utils.coverage_compute_2d(global_speed_steering_cov)
        self.test_dict['whole_suite_speed_steering_entropy'] = utils.entropy_compute_2d(global_speed_steering_cov)
        print("The suite covered: ", self.test_dict['whole_suite_speed_steering_coverage'])

    def road_compare_1d(self, road_to_compare: str, measure: str):
        road_similarities = {}
        main_bin = self.test_dict.get(road_to_compare).get(measure)
        assert main_bin is not None, "The bin " + measure + " has not been added or spelling is incorrect"
        # print(main_bin)
        for i in range(self.start, self.end + 1):
            road_similarities[str(i)] = utils.list_difference_1d(main_bin,
                                                                 self.test_dict.get(str(i)).get(measure),
                                                                 function='binary', normalized=True)
        # print(road_similarities)
        self.test_dict.get(road_to_compare)[road_to_compare + ' ' + measure] = road_similarities

    def road_compare_2d(self, road_to_compare: str, measure: str):
        road_similarities = {}
        main_bin = self.test_dict.get(road_to_compare).get(measure)
        assert main_bin is not None, "The bin " + measure + " has not been added or spelling is incorrect"
        # print(main_bin)
        for i in range(self.start, self.end + 1):
            road_similarities[str(i)] = utils.bin_difference_2d(main_bin,
                                                                self.test_dict.get(str(i)).get(measure),
                                                                function='binary', normalized=True)
        # print(road_similarities)
        self.test_dict.get(road_to_compare)[road_to_compare + ' ' + measure] = road_similarities
