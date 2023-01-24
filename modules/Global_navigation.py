import math
import numpy as np

class GlobalNavigation:

    def __init__(self, start, goal, occupancy_grid, max_valx, max_valy, movement_type, block_size):
        self.block_size = block_size
        self.max_valx = max_valx
        self.max_valy = max_valy
        self.start = start
        self.goal = goal
        self.occupancy_grid = occupancy_grid
        #self.occupancy_grid[occupancy_grid == 1] = 1 # Possible modifications to obtain map containing 0/1
        #self.occupancy_grid[occupancy_grid != 1] = 0
        self.movement_type = movement_type

        # List of all coordinates in the grid
        x, y = np.mgrid[0:self.max_valx:1, 0:self.max_valy:1]
        self.pos = np.empty(x.shape + (2,))
        self.pos[:, :, 0] = x;
        self.pos[:, :, 1] = y
        self.pos = np.reshape(self.pos, (x.shape[0] * x.shape[1], 2))
        self.coords = list([(int(x[0]), int(x[1])) for x in self.pos])

        # Define the heuristic, here = distance to goal ignoring obstacles
        self.h = np.linalg.norm(self.pos - self.goal, axis=-1)
        self.h = dict(zip(self.coords, self.h))

    def get_movements_4n(self):
        """
        Get all possible 4-connectivity movements (up, down, left right).
        :return: list of movements with cost [(dx, dy, movement_cost)]
        """
        return [(1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0)]

    def get_movements_8n(self):
        """
        Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
        (up, down, left, right and the 4 diagonals).
        :return: list of movements with cost [(dx, dy, movement_cost)]
        """
        s2 = math.sqrt(2)
        return [(1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0),
                (1, 1, s2),
                (-1, 1, s2),
                (-1, -1, s2),
                (1, -1, s2)]

    def reconstruct_path(self, cameFrom, current):
        """
        Recurrently reconstructs the path from start node to the current node
        :param cameFrom: map (dictionary) containing for each node n the node immediately
                         preceding it on the cheapest path from start to n
                         currently known.
        :param current: current node (x, y)
        :return: list of nodes from start to current node
        """
        total_path = [current]
        while current in cameFrom.keys():
            # Add where the current node came from to the start of the list
            total_path.insert(0, cameFrom[current])
            current = cameFrom[current]

        # INSERT HERE DESIRED TRANSFORMATIONS FOR total_path


        # print(total_path)

        return total_path

    def A_Star(self):
        """
        A* for 2D occupancy grid. Finds a path from start to goal.
        h is the heuristic function. h(n) estimates the cost to reach goal from node n.
        :param start: start node (x, y)
        :param goal_m: goal node (x, y)
        :param occupancy_grid: the grid map
        :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
        :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
        """

        # Check if the start and goal are within the boundaries of the map
        if (self.start[0] < 0) or (self.start[1] <0) or (self.start[0] >= self.max_valx) or (self.start[1] >= self.max_valy):
            raise Exception('Start node is not contained in the map')

        if (self.goal[0] < 0) or (self.goal[1] < 0) or (self.goal[0] >= self.max_valx) or (self.goal[1] >= self.max_valy):
            raise Exception('Goal node is not contained in the map')


        # check if start and goal nodes correspond to free spaces
        if self.occupancy_grid[self.start[0], self.start[1]]:
            raise Exception('Start node is not traversable')

        if self.occupancy_grid[self.goal[0], self.goal[1]]:
            raise Exception('Goal node is not traversable')

        # get the possible movements corresponding to the selected connectivity
        if self.movement_type == '4N':
            movements = self.get_movements_4n()
        elif self.movement_type == '8N':
            movements = self.get_movements_8n()
        else:
            raise ValueError('Unknown movement')

        # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
        # Initially, only the start node is known.
        openSet = [self.start]

        # The set of visited nodes that no longer need to be expanded.
        closedSet = []

        # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
        cameFrom = dict()

        # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        gScore = dict(zip(self.coords, [np.inf for x in range(len(self.coords))]))
        gScore[self.start] = 0

        # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
        fScore = dict(zip(self.coords, [np.inf for x in range(len(self.coords))]))
        fScore[self.start] = self.h[self.start]

        # while there are still elements to investigate
        while openSet != []:

            # the node in openSet having the lowest fScore[] value
            fScore_openSet = {key: val for (key, val) in fScore.items() if key in openSet}
            current = min(fScore_openSet, key=fScore_openSet.get)
            del fScore_openSet

            # If the goal is reached, reconstruct and return the obtained path
            if current == self.goal:
                return self.reconstruct_path(cameFrom, current), closedSet

            openSet.remove(current)
            closedSet.append(current)

            # for each neighbor of current:
            for dx, dy, deltacost in movements:

                neighbor = (current[0] + dx, current[1] + dy)

                # if the node is not in the map, skip
                if (neighbor[0] >= self.occupancy_grid.shape[0]) or (neighbor[1] >= self.occupancy_grid.shape[1]) or (
                        neighbor[0] < 0) or (neighbor[1] < 0):
                    continue

                # if the node is occupied or has already been visited, skip
                if (self.occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet):
                    continue

                # d(current,neighbor) is the weight of the edge from current to neighbor
                # tentative_gScore is the distance from start to the neighbor through current
                tentative_gScore = gScore[current] + deltacost

                if neighbor not in openSet:
                    openSet.append(neighbor)

                if tentative_gScore < gScore[neighbor]:
                    # This path to neighbor is better than any previous one. Record it!
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    fScore[neighbor] = gScore[neighbor] + self.h[neighbor]

        # Open set is empty but goal was never reached
        print("No path found to goal")
        return [], closedSet


"""
POSSIBLE TRANSFORMATION OPTIONS FOR total_path
total_path = list(np.asarray(total_path) + 0.5) # relative coordinates shifted to the center of the block
total_path = list((np.asarray(total_path) + 0.5) * self.block_size) # transformation of coordinates to be absolute and shifted to the center of the block
total_path = list(np.asarray(total_path) * self.block_size) # transformation to absolute coordinates
"""