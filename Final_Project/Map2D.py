import csv
from Utils import endpoints_to_edges


class Map():

    ''' 
    Map object. A map is generated from a csv while where each row contains the vertices of an obstacle.
        Attributes:
            map_2d : map data imported from a csv file
            obstacles : a list of polygonal obstacles
            obstacle_edges : a list of the line segments which make up all obstacles
        Methods:
            init() : Instantiate a new map object.
            load_map() : Generate a list of obstacles and obstacle edges from data in csv file.
            get_obstacles : Get a list of obstacles and obstacle edges from the map.
    '''

    def __init__(self, file_path):

        '''
        Instantiate a new map object.
            Arguments:
                file_path : the file path for the csv data file which defines the map
        '''

        map_2d = self.load_map(file_path)
        self.obstacles = map_2d[0]
        self.obstacle_edges = map_2d[1]  


    def load_map(self, file_path):
        
        '''
        Generate a list of obstacles and obstacle edges from data in csv file.
            Arguments:
                file_path : the file path for the csv data file which defines the map
            Returns:
                obstacles : a list of polygonal obstacles
                obstacle_edges : a list of the line segments which make up all obstacles
        '''

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

        '''
        Get a list of obstacles and obstacle edges from the map.
            Returns:
                obstacles : a list of polygonal obstacles
                obstacle_edges : a list of the line segments which make up all obstacles
        '''

        return self.obstacles, self.obstacle_edges
