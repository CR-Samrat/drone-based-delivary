import random
import numpy as np
from bluesky import core, stack, traf
import math
from k_means_constrained import KMeansConstrained
from itertools import permutations
from shapely.geometry import Polygon, LineString, Point
import networkx as nx

def init_plugin():
    drone = Adv_drone()

    config = {
        'plugin_name':     'EUCLIDEAN_CAPACITY_K_WITH_OBSTACLE',
        'plugin_type':     'sim',
        }

    return config

# Support functions
def create_locations(x, y, n):
    loc = []

    for _ in range(n):
        loc_x = random.randint(int(x[0]), int(y[0])) + random.randint(int((x[0] - int(x[0]))*10000), int((y[0] - int(y[0]))*10000)) / 10000
        loc_y = random.randint(int(x[1]), int(y[1])) + random.randint(int((x[1] - int(x[1]))*10000), int((y[1] - int(y[1]))*10000)) / 10000
        loc.append([loc_x,loc_y])
        
    return loc

def create_polygon(points):
    pt1 = points[0]
    pt2 = points[1]
    coord = [[pt1[0],pt1[1]], [pt2[0],pt1[1]], [pt2[0], pt2[1]], [pt1[0], pt2[1]], [pt1[0],pt1[1]]]
    obstacle = Polygon(coord)
    return obstacle

def verify_locations(delivery_loc, obstacles):
    obstacle_polygon = []
    answer = []
    for obs_id, obs in obstacles.items():
        obstacle_polygon.append(create_polygon(obs))
    
    for loc in delivery_loc:
        flag = True
        for obs in obstacle_polygon:
            if Point([loc[0],loc[1]]).intersects(obs):
                flag = False
                break
        if flag:
            answer.append(loc)

    return answer

def KMeans(delivery_loc, n, k):
    n_cluster = n//k if n%k == 0 else n//k + 1

    model = KMeansConstrained(n_clusters=n_cluster, size_min=1, size_max=k)
    model.fit_predict(np.array(delivery_loc))

    labels = model.labels_
    clusters = {label:[] for label in range(n_cluster)}

    for i in range(len(delivery_loc)):
        clusters[labels[i]].append(delivery_loc[i])

    return model.cluster_centers_, clusters

def calculate_dp(centroids):
    x = [i for i,j in centroids]
    y = [j for i,j in centroids]

    loc_x = np.median(x)
    loc_y = np.median(y)

    return [loc_x, loc_y]

def calculate_distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.sqrt(np.sum((point1 - point2)**2))

def is_obstacle_free(line, obstacles):
    for obs in obstacles:
        if line.intersects(obs):
            return False
    return True

def create_shortest_path(point_1, point_2, obstacles):
    route_line = LineString([point_1, point_2])

    intersections = []
    for obstacle in obstacles:
        for i in range(len(obstacle.exterior.coords) - 1):
            start, end = obstacle.exterior.coords[i], obstacle.exterior.coords[i + 1]
            line = LineString([start, end])
            if route_line.intersects(line):
                intersections.append(start)
                intersections.append(end)

    # Create a graph
    G = nx.Graph()

    # Add nodes for each point in the space
    G.add_node(tuple(point_1))

    for i in range(len(intersections)):
        point = intersections[i]
        pt_1 = (point[0] + 0.001, point[1])
        pt_2 = (point[0] - 0.001, point[1])
        pt_3 = (point[0], point[1] - 0.001)
        pt_4 = (point[0], point[1] + 0.001)
        pts = [pt_1, pt_2, pt_3, pt_4]

        for pt in pts:
            flag = True
            for obs in obstacles:
                if Point(pt[0], pt[1]).intersects(obs):
                    flag = False
                    break
            if flag:
                G.add_node(pt)
    
    G.add_node(tuple(point_2))

    # Add edges between adjacent points (up, down, left, right)
    for cur in G.nodes():
        for next in G.nodes():
            if cur == next:
                continue
            line = LineString([cur, next])
            if is_obstacle_free(line, obstacles):
                G.add_edge(cur, next)

    # Find the shortest path using A*
    start = tuple(point_1)
    goal = tuple(point_2)
    path = nx.astar_path(G, start, goal, heuristic=calculate_distance)

    return path

def check_obstacle_in_line(point_a, point_b, obstacles):
  line = LineString([point_a, point_b])

  for obstacle in obstacles:
    if line.intersects(obstacle):
      # intermediate_point = find_inter_point(point_a, point_b, obstacles)
      intermediate_point = create_shortest_path(point_a, point_b, obstacles)
      distance = 0

      for pts in range(len(intermediate_point)-1):
        distance += calculate_distance(intermediate_point[pts], intermediate_point[pts + 1])

      return distance, intermediate_point

  return calculate_distance(point_a, point_b), None

def create_distance_matrix(points,dp,obstacles):
    matrix = {}
    inter_matrix = {}
    obstacle_polygon = []
    for obs_id, obs in obstacles.items():
        obstacle_polygon.append(create_polygon(obs))

    points.append(dp)
    for src in points:
        for dest in points:
            src_point = tuple(src)
            dest_point = tuple(dest)
            distance,inter = check_obstacle_in_line(src_point, dest_point, obstacle_polygon)
            matrix[(src_point, dest_point)] = distance
            if inter != None:
                inter_matrix[(src_point, dest_point)] = inter
    
    return matrix, inter_matrix

def tsp_brut(cluster, dp, obstacles):
    paths = list(permutations(cluster))
    distance_matrix, inter_point_matrix = create_distance_matrix(cluster,dp, obstacles)
    distance_list = []

    # work start
    for cur in paths:
        path = list(cur)
        path.append(dp)
        path.insert(0, dp)

        # distance calculation
        distance = 0
        for loc in range (len(path)-1):
            distance += distance_matrix[(tuple(path[loc]), tuple(path[loc + 1]))]
        distance_list.append(distance)

    # getting the minimum distance and path
    min_path = list(paths[np.argmin(distance_list)])
    min_path.append(dp)
    min_path.insert(0,dp)
    min_dist = distance_list[np.argmin(distance_list)]

    #insert intermediate points
    ans_route = []
    for loc in range(len(min_path)-1):
        ans_route.append(min_path[loc])
        if inter_point_matrix.get((tuple(min_path[loc]), tuple(min_path[loc + 1]))):
            intermediate_points =  inter_point_matrix[(tuple(min_path[loc]), tuple(min_path[loc + 1]))]
            for i in range(1,len(intermediate_points)-1):
                ans_route.append(intermediate_points[i])
    ans_route.append(dp)

    print("Optimal path",min_path,"optimal distance",min_dist)

    return ans_route

# Main program
class Adv_drone(core.Entity):
    def __init__(self):
        super().__init__()

        stack.stack(f'BOX Eucledian 22.5726 88.4010 22.6141 88.4654')
        stack.stack(f'BOX Obstacles1 22.5801 88.4110 22.5841 88.4190')
        stack.stack(f'BOX Obstacles2 22.6042 88.4210 22.6082 88.4290')
        stack.stack(f'BOX Obstacles3 22.6084 88.4410 22.6124 88.4490')
        stack.stack(f'BOX Obstacles4 22.5798 88.4510 22.5838 88.4590')
        stack.stack(f'BOX Obstacles5 22.5847 88.4390 22.5887 88.4470')

        self.n = 10
        self.k = 3
        self.x = [22.5726, 88.4010]
        self.y = [22.6141, 88.4654]

        self.obstacles = {
            0 : [[22.5801, 88.4110],[22.5841, 88.4190]],
            1 : [[22.6042, 88.4210],[22.6082, 88.4290]],
            2 : [[22.6084, 88.4410],[22.6124, 88.4490]],
            3 : [[22.5798, 88.4510],[22.5838, 88.4590]],
            4 : [[22.5847, 88.4390],[22.5887, 88.4470]],
        }

        self.delivary_loc = create_locations(self.x, self.y, self.n)
        self.centroids = []
        self.clusters = dict()
        self.dp = []

    @stack.command
    def create_drone(self, acid: str, lat:float, lon:float):
        stack.stack(f'CRE {acid} A225 {lat} {lon} 130 FL410 485')

    @stack.command
    def map_delivery_locations(self):
        idx = 0
        self.delivary_loc = verify_locations(self.delivary_loc, self.obstacles)
        for loc in self.delivary_loc:
            idx+=1
            stack.stack(f'DEFWPT T-{idx} {loc[0]} {loc[1]}')

    @stack.command
    def map_dp(self):
        self.centroids, self.clusters = KMeans(self.delivary_loc, self.n, self.k)
        self.dp = calculate_dp(self.centroids)
        stack.stack(f'DEFWPT DP {self.dp[0]} {self.dp[1]}')

    @stack.command
    def create_route(self):
        for i in self.clusters:
            self.clusters[i] = tsp_brut(self.clusters[i], self.dp, self.obstacles)
        for i in self.clusters:
            stack.stack(f'ECHO {self.clusters[i]}')

    @stack.command
    def start_delivery(self):
        stack.stack(f'CRE drone B105 DP 0 10 10')

        for i in self.clusters:
            locs = self.clusters[i]
            cur_pt = self.dp
            for j in locs:
                nxt_pt = j
                if cur_pt == nxt_pt:
                    continue

                stack.stack(f'ADDWPTMODE drone FLYOVER')
                stack.stack(f'ADDWPT drone {j[0]} {j[1]} 0 0')

                cur_pt = nxt_pt