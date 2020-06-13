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
        """ Calculates the 2d coverage of steering across the whole test suite
            The coverage is added to the test suite dictionary

        :return: Coverage across the suite
        """
        global_speed_steering_cov = np.zeros((NUM_BINS, NUM_BINS))
        for i in range(self.start, self.end + 1):
            cov_col = self.test_dict.get(str(i)).get('cov_collector')
            global_speed_steering_cov = np.add(global_speed_steering_cov, cov_col.get_speed_steering_2d())
        self.test_dict['whole_suite_speed_steering_coverage'] = utils.coverage_compute_2d(global_speed_steering_cov)
        self.test_dict['whole_suite_speed_steering_entropy'] = utils.entropy_compute_2d(global_speed_steering_cov)
        print("The suite covered: ", self.test_dict['whole_suite_speed_steering_coverage'])
        return utils.coverage_compute_2d(global_speed_steering_cov)

    def calculate_suite_coverage_1d(self, feature: str):
        """ Calculates the 1d coverage of a selected feature across the whole test suite
            The coverage is added to the test suite dictionary

        :param feature: the feature name across the coverage is calculated
        :return: Coverage across the suite
        """
        global_bins = [0] * NUM_BINS
        for i in range(self.start, self.end + 1):
            cov_col = self.test_dict.get(str(i)).get(feature)
            assert cov_col is not None, "The bin " + feature + " has not been added or spelling is incorrect"
            global_bins = np.add(global_bins, cov_col)
        cov = utils.coverage_compute_1d(global_bins)
        self.test_dict["whole_suite_" + feature + "_coverage"] = cov
        return cov

    def road_compare_1d(self, road_to_compare: str, measure: str):
        """ compares the coverage of a single-dimensional feature of a road to all others in the suite

        :param road_to_compare: the baseline road which is compared to all others
        :param measure: the feature which is compare, has to be present for each road in the suite dict
        :return: None
        """
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
        """ compares the coverage of a two-dimensional feature of a road to all others in the suite

        :param road_to_compare: the baseline road which is compared to all others
        :param measure: the feature which is compare, has to be present for each road in the suite dict
        :return: None
        """
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
