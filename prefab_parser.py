import re
from shapely.geometry import LineString
import matplotlib.pyplot as plt

PLOT_VISIBLE = False

class PrefabParser:
    def __init__(self, file_name: str):
        self.file_name = file_name
        self.nodes = []
        self.road_width = 0

    def parse_road(self, road_name: str):
        """ parses the prefab of a scenario and extracts the trajectory of a selected road

        :param road_name: name (not id!) of the road as string
        :return: LineString of the road
        """
        with open(self.file_name, 'r') as prefab_file_object:
            prefab_file = prefab_file_object.read()
            road_search = re.search(road_name + r'\).*?\};', prefab_file, re.DOTALL)  # re.search('road_name\)(.*)\};', prefab_file)
            assert road_search is not None, "The road has not been found in the prefab!"
            #print(road_search.group(0))
            #line = road_search.group(0).readline()
            all_nodes = re.findall(r'Node.*?;', road_search.group(0))

            for node in all_nodes:
                numeric_const_pattern = r'[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
                rx = re.compile(numeric_const_pattern, re.VERBOSE)
                extracted_coordinates = rx.findall(node)
                assert extracted_coordinates.__len__() == 4, "Wrong number of coordinates found"
                node_point = (float(extracted_coordinates[0]), float(extracted_coordinates[1]))
                self.nodes.append(node_point)

                if self.road_width == 0:
                    self.road_width = float(extracted_coordinates[3])

            lstr = LineString(self.nodes)
            if PLOT_VISIBLE:
                x, y = lstr.xy
                plt.plot(x, y)
                plt.show()
            return lstr

    def get_road_width(self):
        return self.road_width
