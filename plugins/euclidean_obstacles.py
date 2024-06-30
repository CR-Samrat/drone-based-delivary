import random
import numpy as np
from bluesky import core, stack, traf
import math
from k_means_const_manhattan import KMeansConstrained
from itertools import permutations
from shapely.geometry import Point, Polygon, LineString
import networkx as nx

def init_plugin():
    drone = Adv_drone()

    config = {
        'plugin_name':     'EUCLIDEAN_OBSTACLES',
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
#remove_obstacle(delivary_loc, obstacles)
def create_polygon(points):
    pt1 = points[0]
    pt2 = points[1]
    coord = [[pt1[0],pt1[1]], [pt2[0],pt1[1]], [pt2[0], pt2[1]], [pt1[0], pt2[1]], [pt1[0],pt1[1]]]
    obstacle = Polygon(coord)
    return obstacle
def remove_obstacle(delivary_loc, obstacles):
    obstacle_polygon = {obs_id : create_polygon(obs) for obs_id, obs in obstacles.items()}
    delivary_point = [Point([a,b]) for a,b in delivary_loc]
    final_point = []
    
    for point in delivary_point:
        flag = True
        for obs_id, obs in obstacle_polygon.items():
            if (point.intersects(obs)):
                flag = False
                break
        if flag:
            final_point.append([point.x, point.y])
    return final_point


def create_route(pts, extra):
    lst = pts.copy()
    idx = 0
    k = len(lst)
    for i in range(k):
        lst.insert(idx, extra)
        idx +=2

    lst.append(extra)
    return lst
def calculate_dp(delivary_loc):
    x = [i for i,j in delivary_loc]
    y = [j for i,j in delivary_loc]

    x_axis = np.mean(x)
    y_axis = np.mean(y)

    return [x_axis, y_axis]


# route
# THRESHOLD = initial_dist + (initial_dist*0.03)

# from shapely.geometry import LineString, Polygon, Point

def check_obstacle_in_line(point_a, point_b, obstacles):
  line = LineString([point_a, point_b])

  for obstacle_id, points in obstacles.items():
    pt1 = points[0]
    pt2 = points[1]
    coord = [[pt1[0],pt1[1]], [pt2[0],pt1[1]], [pt2[0], pt2[1]], [pt1[0], pt2[1]], [pt1[0],pt1[1]]]
    obstacle = Polygon(coord)
    if line.intersects(obstacle):
      return True

  return False

def detect_obstacle(route, obstacles):
    for i in range(0,len(route)-1,2):
        if check_obstacle_in_line(route[i],route[i+1], obstacles):
            return True
    return False
def adv_route(route, dp, obstacles):
    ans_route = []
    for i in range(len(route)-1):
        if route[i] != dp:
            continue
        if check_obstacle_in_line(route[i],route[i+1], obstacles):
            intermediate_point = find_inter_point(route[i], route[i+1], obstacles)
            if intermediate_point == None:
                return route, "fail"
            ans_route.append(route[i])
            ans_route.append(intermediate_point)
            ans_route.append(route[i+1])
            ans_route.append(intermediate_point)
        else:
            ans_route.append(route[i])
            ans_route.append(route[i+1])

    ans_route.append(dp)
    return ans_route, "pass"


import math
def is_obstacle_free(line, obstacles):
    for obs_id, obs in obstacles.items():
        if line.intersects(obs):
            return False
    return True

def find_inter_point(point_a, point_b, obstacles):
    route_line = LineString([point_a, point_b])
    obstacle_polygons = {id_: create_polygon(points) for id_, points in obstacles.items()}
    target_obs = None

    # Find all intersections with obstacles (including edges)
    intersections = []
    for obstacle_id, obstacle in obstacle_polygons.items():
        for i in range(len(obstacle.exterior.coords) - 1):
            start, end = obstacle.exterior.coords[i], obstacle.exterior.coords[i + 1]
            line = LineString([start, end])
            if route_line.intersects(line):
                intersections.append(route_line.intersection(line))
            if target_obs == None and len(intersections)>0:
                target_obs = obstacle

    # If no intersections found, return None
    if not intersections:
        return None

    # Choose the intersection with the smallest sum of distances to A and B
    inter_x = sum(inter.x for inter in intersections)/len(intersections)
    inter_y = sum(inter.y for inter in intersections)/len(intersections)
    nearest_inter = Point(inter_x, inter_y)

    interval = 0.001

    while interval < 0.1:
        # Check if there's a clear path along either side of the obstacle
        left_side = LineString([point_a, Point(nearest_inter.x - interval, nearest_inter.y)])
        right_side = LineString([point_a, Point(nearest_inter.x + interval, nearest_inter.y)])
        up_side = LineString([point_a, Point(nearest_inter.x , nearest_inter.y + interval)])
        down_side = LineString([point_a, Point(nearest_inter.x , nearest_inter.y - interval)])

        # check if left_side is obstacle free
        # if not left_side.intersects(target_obs):
        if is_obstacle_free(left_side, obstacle_polygons):
            left_to_B = LineString([[nearest_inter.x - interval, nearest_inter.y],[point_b[0],point_b[1]]])
            flag_1 = True
            for obs_id, obs in obstacle_polygons.items():
                if(left_to_B.intersects(obs)):
                    flag_1 = False
                    break
            if flag_1:
                return [nearest_inter.x - interval, nearest_inter.y]
        # check if right_side is obstacle free
        # if not right_side.intersects(target_obs):
        if is_obstacle_free(right_side, obstacle_polygons):
            right_to_B = LineString([[nearest_inter.x + interval, nearest_inter.y],[point_b[0],point_b[1]]])
            flag_2 = True
            for obs_id, obs in obstacle_polygons.items():
                if(right_to_B.intersects(obs)):
                    flag_2 = False
                    break
            if flag_2:
                return [nearest_inter.x + interval, nearest_inter.y]
        # check if up_side is obstacle free
        # if not up_side.intersects(target_obs):
        if is_obstacle_free(up_side, obstacle_polygons):
            up_to_B = LineString([[nearest_inter.x, nearest_inter.y + interval],[point_b[0],point_b[1]]])
            flag_3 = True
            for obs_id, obs in obstacle_polygons.items():
                if(up_to_B.intersects(obs)):
                    flag_3 = False
                    break
            if flag_3:
                return [nearest_inter.x, nearest_inter.y + interval]
        # check if down_side is obstacle free
        # if not down_side.intersects(target_obs):
        if is_obstacle_free(down_side, obstacle_polygons):
            down_to_B = LineString([[nearest_inter.x, nearest_inter.y - interval],[point_b[0],point_b[1]]])
            flag_4 = True
            for obs_id, obs in obstacle_polygons.items():
                if(down_to_B.intersects(obs)):
                    flag_4 = False
                    break
            if flag_4:
                return [nearest_inter.x, nearest_inter.y - interval]
        interval += 0.001

    return None

# A* Algorithm

def create_shortest_path(point_1, point_2, obstacles):
    obstacle_polygon = {ob_id : create_polygon(obs) for ob_id, obs in obstacles.items()}
    route_line = LineString([point_1, point_2])

    intersections = []
    for obstacle_id, obstacle in obstacle_polygon.items():
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
            for obs_id, obs in obstacle_polygon.items():
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
            if is_obstacle_free(line, obstacle_polygon):
                G.add_edge(cur, next)

    # Find the shortest path using A*
    start = tuple(point_1)
    goal = tuple(point_2)
    try:
        path = nx.astar_path(G, start, goal, heuristic=euc_distance)
    except:
        path = None

    return path

def a_star_algo(route, dp, obstacles):
    ans_route = []
    for i in range(len(route)-1):
        if route[i] != dp:
            continue
        if check_obstacle_in_line(route[i],route[i+1], obstacles):
            new_path = create_shortest_path(route[i], route[i+1], obstacles)
            if new_path == None:
                return route, "fail"
            for i in new_path:
                ans_route.append(i)
            for i in range(len(new_path)-2, 0, -1):
                ans_route.append(new_path[i])
        else:
            ans_route.append(route[i])
            ans_route.append(route[i+1])

    ans_route.append(dp)
    return ans_route, "pass"


#distance
def total_dist(pts):
    sum = 0
    for i in range(len(pts)-1):
        sum += euc_distance(pts[i], pts[i+1])

    return sum
def euc_distance(pt1, pt2):
    pt1 = np.array(pt1)
    pt2 = np.array(pt2)
    return np.sqrt(np.sum((pt2 - pt1)**2))

# calculate obstacle free DP
def calculate_obstacle_free_dp(delivery_loc, obstacles, dp):

    # Initialize potential DP locations (covering the delivery point area)
    min_x = dp[0] - 0.1
    max_x = dp[0] + 0.1
    min_y = dp[1] - 0.1
    max_y = dp[1] + 0.1

    potential_dps = {}
    for x in frange(min_x, max_x, 0.01):
        for y in frange(min_y, max_y, 0.01):
            potential_dps[x,y] = {
                'route' : [],
                'distance' : 0
            }

    # Evaluate each potential DP and filter out those obstructed by obstacles
    for each_dp, value in potential_dps.items():
        first_route = create_route(delivery_loc, each_dp)
        last_route,msg = a_star_algo(first_route,each_dp,obstacles)
        if msg == 'pass':
            distance = total_dist(last_route)
        else:
            distance = float('inf')
        potential_dps[each_dp]['route'] = last_route
        potential_dps[each_dp]['distance'] = distance

    answer = min(potential_dps, key = lambda x : potential_dps[x]['distance'])

    return answer,potential_dps[answer]['route'],potential_dps[answer]['distance']

def frange(start, stop, step):
    while start < stop:
        yield round(start, 4)
        start += step


# Main program
class Adv_drone(core.Entity):
    def __init__(self):
        super().__init__()

        stack.stack(f'BOX Eucledian 22.5726 88.4010 22.6141 88.4654')
        stack.stack(f'BOX Obstacles1 22.5826 88.4020 22.5941 88.4054')
        stack.stack(f'BOX Obstacles2 22.5986 88.4120 22.6021 88.4170')
        stack.stack(f'BOX Obstacles3 22.6026 88.4420 22.6084 88.4454')
        stack.stack(f'BOX Obstacles4 22.5740 88.4540 22.5790 88.4594')
        self.n = 10
        self.k = 5
        self.x = [22.5726, 88.4010]
        self.y = [22.6141, 88.4654]
        self.obstacles = {
            0: [[22.5826, 88.4020], [22.5941, 88.4054]],
            1: [[22.5986, 88.4120], [22.6021, 88.4170]],
            2: [[22.6026, 88.4420], [22.6084, 88.4454]],
            3: [[22.5740, 88.4540], [22.5790, 88.4594]]
        }
        self.delivary_loc = create_locations(self.x, self.y, self.n)
        # self.centroids, self.clusters = KMeans(self.delivary_loc, self.n, self.k)
        # self.dp = calculate_dp(self.centroids)
    
    @stack.command
    def map_delivery_locations(self):
        idx = 0
        self.delivary_loc = remove_obstacle(self.delivary_loc, self.obstacles)
        for loc in self.delivary_loc:
            idx+=1
            stack.stack(f'DEFWPT T-{idx} {loc[0]} {loc[1]}')


    @stack.command
    def map_DP(self):
        self.dp = calculate_dp(self.delivary_loc)
        # stack.stack(f'DEFWPT DP {self.dp[0]} {self.dp[1]}')
        self.initial_route = create_route(self.delivary_loc, self.dp)
        # stack.stack(f'ECHO {self.initial_route}')
        if(detect_obstacle(self.initial_route, self.obstacles)):
            temp_route = self.initial_route.copy()
            new_route, msg = a_star_algo(temp_route, self.dp, self.obstacles)
            if msg == "pass":
                new_dist = total_dist(new_route)
            else:
                 new_dist = float('inf')
            # stack.stack(f'DEFWPT DP {self.dp[0]} {self.dp[1]}')
            optimal_route = new_route
            optimal_dp = self.dp

            # new_dp,final_route,final_dist = calculate_obstacle_free_dp(self.delivary_loc, self.obstacles, self.dp)

            #   # comparison
            # if final_dist < new_dist:
            #     optimal_dp = new_dp
            #     optimal_route = final_route
            # else:
            #     optimal_dp = self.dp
            #     optimal_route = new_route

            stack.stack(f'DEFWPT DP {optimal_dp[0]} {optimal_dp[1]}')
            self.initial_route = optimal_route

        else: 
            stack.stack(f'DEFWPT DP {self.dp[0]} {self.dp[1]}')
    @stack.command
    def Start_delivery(self):
        stack.stack(f'CRE drone B105 DP 0 10 10')
        for route in self.initial_route:
            x=route[0]
            y=route[1]
            stack.stack(f'ADDWPTMODE drone FLYOVER')
            stack.stack(f'ADDWPT drone {x} {y} 0 0')
            stack.stack(f'ADDWPTMODE drone FLYOVER')