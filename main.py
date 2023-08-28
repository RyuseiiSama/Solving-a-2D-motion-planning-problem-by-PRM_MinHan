# -------- Libraries ------------
import heapq
import random
import numpy as np
import pylab as pl
from matplotlib import pyplot as plt
import environment_2d


# ---------- PRM PARAMETERS --------------
N = 10000  # Number of random nodes
REFINE = 10 # Repetitions for post processing
MIN_R = 3
X_SIZE = 20
Y_SIZE = 20
N_OBSTACLES = 6 # Number of obstacles

# -------------Edge checking class -----------

class EdgeChecker:  # To check if straight edges cut through obstacles
    def __init__(self, obs):
        self.obstacles = obs
        self.obslines = []
        for ob in self.obstacles:  # Save all obstacles as lines
            self.obslines.append(((ob.x0, ob.y0), (ob.x1, ob.y1)))
            self.obslines.append(((ob.x1, ob.y1), (ob.x2, ob.y2)))
            self.obslines.append(((ob.x2, ob.y2), (ob.x0, ob.y0)))

    def intersect(self, edge: tuple):
        """Checks whether the edge intersects with obstacle. This is done by checking if the line segments intersect
        at a point x, and whether that x value lies in overlapping X-intervals of the line segment"""
        # y = Ax + b
        try:
            x1_edge = edge[0][0]
            y1_edge = edge[0][1]
            x2_edge = edge[1][0]
            y2_edge = edge[1][1]
            A_edge = (y2_edge - y1_edge) / (x2_edge - x1_edge)
            b_edge = y1_edge - A_edge * x1_edge
        except ZeroDivisionError:
            return True
        for l in self.obslines:
            x1_ob = l[0][0]
            y1_ob = l[0][1]
            x2_ob = l[1][0]
            y2_ob = l[1][1]
            Ia = [max(min(x1_edge, x2_edge), min(x1_ob, x2_ob)),
                  min(max(x1_edge, x2_edge), max(x1_ob, x1_ob))]
            if Ia[0] > Ia[1]:  # If no intervals exist, no intersect
                continue
            A_l = (y2_ob - y1_ob) / (x2_ob - x1_ob)
            b_l = y1_ob - A_l * x1_ob
            intersect_x = (b_l - b_edge) / (A_edge - A_l)
            if Ia[0] < intersect_x < Ia[1]:
                return True

        return False


# --------- PRM Algorithm ---------------

class PRM_algo():
    def __init__(self, obs):
        self.points = []  # List of probabilistically generated points
        self.edges = {}  # Dictionary to store all lines in the form {point:[(neighbour,distance)]}
        self.check = EdgeChecker(obs)

    def eucil_length(self, point1, point2):
        """Returns edge length"""
        length = ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
        return length

    def random_point(self):
        """Returns randomly generated point"""
        x = round(random.random() * X_SIZE, 6)
        y = round(random.random() * Y_SIZE, 6)
        if env.check_collision(x, y):
            return self.random_point()

        return x, y

    def locate_neigbours(self, point: tuple):
        """Find neigbours"""
        neighbours = [self.points[i] for i in range(len(self.points)) if
                      self.eucil_length(point, self.points[i]) < MIN_R and self.points[i] != point]
        if not neighbours:  # If no near neighbours
            return 0
        return neighbours

    def run_PRM(self, start, end):
        self.points.append(start)
        self.points.append(end)
        for _ in range(N):  # Generate all sampled points
            self.points.append(self.random_point())

        i = 0
        for point in self.points:
            i += 1

            neighbour_list = self.locate_neigbours(point)
            if neighbour_list == 0:
                continue
            for neighbour in neighbour_list:

                if self.check.intersect((point, neighbour)):
                    continue
                self.edges.setdefault(point, []).append((neighbour, self.eucil_length(point,
                                                                                      neighbour)))  # Check if the point already has edges, if not create new
        print("Roadmap generated.")
    def plot(self):

        # Plot edges between connected nodes
        for node, neighbors in self.edges.items():
            for neighbor in neighbors:
                plt.plot([node[0], neighbor[0][0]], [node[1], neighbor[0][1]], 'b')
        # Plot sample nodes
        pl.scatter(*zip(*self.points), c='r', marker='o', label='Samples', s=2)
        pl.draw()

    def find_shortest_path(self, start, end):
        """ Uses A* method to find nearest path"""
        print("Finding shortest path...")

        heap = []
        heapq.heappush(heap, (1000, start))  # Insert start node, with heuristic value as priority
        parent_node = {}  # This dictionary will hold {child point:parent point}
        cost_to_node = {start: 0}  # Cost allocation for each node (cost only)

        while heap:

            current = heapq.heappop(heap)  # Look for lowest cost node amongst all nodes explored
            if current[1] == end:  # Means reached
                break

            for node in self.edges.get(current[1], []):
                cost = cost_to_node[current[1]] + node[1]

                if node[0] not in cost_to_node or cost < cost_to_node[
                    node[0]]:  # if node has no cost associated or new cost is lower
                    cost_to_node[node[0]] = cost  # Assign lowest cost to current node
                    priority = cost + self.eucil_length(node[0], end)
                    parent_node[node[0]] = current[1]
                    heapq.heappush(heap, (priority, node[0]))
        # Make path
        path = [end]
        while path[-1] != start:
            node = parent_node.get(path[-1])
            path.append(node)
            if node == None:
                print("No path found")
                raise ValueError  # Path not found
        print("Path found! Plotting...")
        path = self.post_process(path)
        return path

    def plot_path(self, path):

        # Plot edges between connected nodes

        x, y = zip(*path)
        plt.plot(x, y)
        pl.scatter(*zip(*path), c='r', marker='o', label='Samples', s=2)
        pl.draw()

    def post_process(self,path):
        if len(path) == 1:
            return path
        print(path)
        for i in range(REFINE):
            a = random.randint(0,len(path)-1)
            b = random.randint(0,len(path)-1)
            if a == b:
                continue
            if self.check.intersect((path[a], path[b])) == False:
                if a>b:
                    print(a,b)
                    del path[b+1:a-1]
                elif a<b:
                    print(a,b)
                    del path[a+1:b-1]
        print(path)
        return path






# --------- Generating Problem Environment -----------
# Start is blue square, goal is gold star
pl.ion()
np.random.seed(4)
env = environment_2d.Environment(X_SIZE, Y_SIZE, N_OBSTACLES)
pl.clf()
env.plot()

q = env.random_query()
if q is not None:
    x_start, y_start, x_goal, y_goal = q
    env.plot_query(x_start, y_start, x_goal, y_goal)

# ------ Run ---------
prm = PRM_algo(env.obs)
prm.run_PRM((x_start, y_start), (x_goal, y_goal))
# prm.plot() # Use this to visualise points generated by PRM
path = prm.find_shortest_path((x_start, y_start), (x_goal, y_goal))
prm.plot_path(path) # Use this to visualise path


pl.pause(12313)
