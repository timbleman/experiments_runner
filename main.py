"""This module runs BeamNG files und collects data locally."""

from pathlib import Path

from beamngpy import BeamNGpy, Scenario, Vehicle

from coverage_collector import CoverageCollector
import utils
bng = BeamNGpy('localhost', 64256)

FIRST_TEST = 96
LAST_TEST = 98

tests_dict = {}

vehicle = Vehicle('ego', model='etk800', licence='PYTHON')

for i in range(FIRST_TEST, LAST_TEST):
    cov_collector = CoverageCollector(vehicle, bng)
    # Load different scenarios.
    scenario = Scenario('drivebuild', 'drivebuild_' + str(i))
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))

    # Has to be the correct folder.
    destination_path = 'C:\\Users\\fraun\\Documents\\BeamNG\\levels\\drivebuild\\scenarios'
    scenario.path = Path(destination_path)

    bng.open()
    bng.load_scenario(scenario)
    bng.start_scenario()

    vehicle.ai_drive_in_lane(True)
    vehicle.ai_set_speed(20, "limit")
    vehicle.ai_set_mode("span")

    # Data collecting loop. Collects every three steps data.
    counter = 0
    last_counter = 0
    steps = 2
    while counter < 100:
        if counter > last_counter + steps:
            cov_collector.collect()
        counter += 1
        if counter % 10 == 0:
            print(counter)
    # bng.stop_scenario() # not working?
    # bng.restart_scenario()
    bng.close()
    # adds binned behavior to dict of road
    coverage = {'steering': cov_collector.get_steering_bins(), 'throttle': cov_collector.get_throttle_bins(),
                'speed_steering': cov_collector.get_speed_steering_2d()}
    print("type: ", type(coverage['speed_steering']))
    print("entropy steering: ", utils.entropy_compute_1d(coverage['steering']))
    print("2d diff: ", utils.bin_difference_2d(coverage['speed_steering'], coverage['speed_steering'], 'binary', False))
    # adds the dictionary of the current road to the global one
    tests_dict[str(i)] = coverage

print(utils.list_difference_1d([2,6,7], [1,6,8], 'squared', normalized=True))
