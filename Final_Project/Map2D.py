import csv
from Utils import endpoints_to_edges


class Map():
    def __init__(self, file_path):
        map_2d = self.load_map(file_path)
        # list of obstacles
        self.obstacles = map_2d[0]
        # list of obstacle edges including map boundary
        self.obstacle_edges = map_2d[1]  


    def load_map(self, file_path):
        # Map container
        obstacles = []
        obstacle_edges = []

        # Load from the csv file
        # The rows are obstacle vertices
        with open(file_path, 'r') as map_file:
            reader = csv.reader(map_file)
            # load the obstacles
            for i, row in enumerate(reader):
                obstacle = []
                # load (x, y) as obstacle vertices
                for j in range(0, len(row), 2):
                    if row[j] == '' or row[j + 1] == '':
                        break
                    point = (float(row[j]), float(row[j + 1]))
                    obstacle.append(point)
                obstacles.append(obstacle)

        # Build the edges of obstacles
        for obstacle in obstacles:
            each_edges = endpoints_to_edges(obstacle, closed=True)
            obstacle_edges.extend(each_edges)

        return obstacles, obstacle_edges


    def get_obstacles(self):
        return self.obstacles, self.obstacle_edges
