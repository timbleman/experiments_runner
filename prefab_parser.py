import re
from shapely.geometry import LineString
import matplotlib.pyplot as plt

PLOT_VISIBLE = True

class PrefabParser:
    def __init__(self, file_name: str):
        self.file_name = file_name
        self.nodes = []

    def parse_road(self, road_name: str):
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
            print(self.nodes)

            lstr = LineString(self.nodes)
            if PLOT_VISIBLE:
                x, y = lstr.xy
                plt.plot(x, y)
                plt.show()
            return lstr
