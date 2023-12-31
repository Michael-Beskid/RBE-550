# Basic searching algorithms
from util import Stack, Queue, PriorityQueueWithFunction, PriorityQueue
import numpy as np

# Class for each node in the grid
class Node:
    def __init__(self, row, col, cost, parent, goal):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.cost = cost      # total cost (depend on the algorithm)
        self.goal = goal      # Goal   
        self.parent = parent  # previous node


# Class for grid dimensions
class gridSize:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols


# Compute number of rows and columns in grid
def calcGridSize(grid):
    gridRows = len(grid[0])
    gridCols = len(grid)
    return gridSize(gridRows, gridCols)


# Check is a node is located at the goal position
def isGoalNode(node):
    return (node.row, node.col) == (node.goal[0], node.goal[1])


# Check is a neighboring square is a valid node
def isValidNode(grid, gridSize, node, V):
    # Confirm that square lies within the grid
    if node.row >= 0 and node.row < gridSize.rows:
        if node.col >= 0 and node.col < gridSize.cols:
            # Confirm that square doesn't contain an obstacle
            if grid[node.row][node.col] == 0:
                # Confirm that square hasn't been visited or discovered yet
                for visitedNode in V:
                    if visitedNode.row == node.row and visitedNode.col == node.col:
                        return False
                # Return true if new node passes all checks
                return True
    return False


# Add valid, unvisited neighbors to queue
def addNeighbors(node, nodeList, neighborOrder):

    # Add neighboring squares to node list
    for order in neighborOrder:
        neighborNode = Node(node.row + order[0], node.col + order[1], None, node, node.goal)
        neighborNode.cost = calcCost(neighborNode)
        nodeList.push(neighborNode)

    # return updated queue
    return nodeList


# Create list of nodes in path from start to goal
def calcPath(path, node):
    if node.parent != None:
        path.insert(0,[node.row, node.col])
        calcPath(path, node.parent)
    else:
        path.insert(0,[node.row, node.col])
    return path


# Calculate the number of nodes evaluated to find a path
def calcNumSteps(V, goal):
    for visitedNode in V:
        if visitedNode.row == goal[0] and visitedNode.col == goal[1]:
            return V.index(visitedNode) + 1
    else:
        return -1
    

# Compute Manhattan distance between a node and the goal
def calcManhattanDistance(node):
    return abs(node.row - node.goal[0]) + abs(node.col - node.goal[1])


# Compute cost for node
def calcCost(node):
    if node.parent != None:
        return node.parent.cost + calcManhattanDistance(node)
    else:
        return calcManhattanDistance(node)
    

# General pathfinding solution for any search algorithm
def findPath(grid, start, goal, nodeList, neighborOrder):

    path = []
    steps = 0
    found = False
    V = []

    # get grid dimensions
    gridSize = calcGridSize(grid)

    # Add start node to queue
    startNode = Node(start[0], start[1], 0, None, goal)
    nodeList.push(startNode)

    while not nodeList.isEmpty():

        currNode = nodeList.pop()

        if isValidNode(grid, gridSize, currNode, V):

            V.append(currNode)

            # break loop and calculate path if goal is reached
            if isGoalNode(currNode):
                path = calcPath(path, currNode)
                steps = calcNumSteps(V, goal)
                found = True
                break
            
            # Add free space neighbors to queue
            nodeList = addNeighbors(currNode, nodeList, neighborOrder)

    return found, path, steps


# Breadth First Search
def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. 
            If no path exists return an empty list [].
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''

    # Initialization
    Q = Queue()
    neighborOrder = [(0,1), (1,0), (0,-1), (-1,0)]

    # Calculate path
    found, path, steps = findPath(grid, start, goal, Q, neighborOrder)

    # Print results
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


# Depth First Search
def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    
    # Initialization
    S = Stack()
    neighborOrder = [(-1,0), (0,-1), (1,0), (0,1)]

    # Calculate path
    found, path, steps = findPath(grid, start, goal, S, neighborOrder)

    # Print results
    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


# A* (huersitic = Manhattan distance)
def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''

    # Initialization
    PQ = PriorityQueueWithFunction(calcCost)
    neighborOrder = [(0,1), (1,0), (0,-1), (-1,0)]

    # Calculate path
    found, path, steps = findPath(grid, start, goal, PQ, neighborOrder)

    # Print results
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
