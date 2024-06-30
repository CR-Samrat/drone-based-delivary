from random import randint
import numpy as np
from bluesky import core, stack, traf
from itertools import permutations
import math

def init_plugin():
    drone = Drone()

    config = {
        'plugin_name':     'DRONE',
        'plugin_type':     'sim',
        }

    return config

def gec(coordinates):
    x_sum = sum(loc[0] for loc in coordinates)
    y_sum = sum(loc[1] for loc in coordinates)
    x_cord = x_sum / len(coordinates)
    y_cord = y_sum / len(coordinates)

    return x_cord, y_cord

def gmm(coordinate):
    x_axis = []
    y_axis = []
    centroid = []

    for i in coordinate:
        x_axis.append(i[0])
        y_axis.append(i[1])

    centroid.append(np.median(x_axis))
    centroid.append(np.median(y_axis))

    return centroid[0],centroid[1]

def ecmb(coordinate, border):
    longitude = 0
    latitude = 0
    k=border
    test = gec(coordinate)
    for loc in coordinate:
        if loc[1] <= k:
            longitude+= loc[1]
        else:
            longitude+= k
        latitude+= loc[0]
    y_axis = longitude/len(coordinate)
    x_axis = latitude/len(coordinate)
    return x_axis, y_axis

def calculate_manhattan_median(customers):
    # Extract coordinates of customer locations
    coordinates = [(x, y) for x, y in customers]

    # Calculate median coordinates for rows and columns
    median_row = sorted([x for x, y in coordinates])[len(coordinates) // 2]
    median_column = sorted([y for x, y in coordinates])[len(coordinates) // 2]

    return median_row, median_column

def mmeb(customer_locations, border_position):
    R = border_position  

    euclidean_locations = [(x, y) for x, y, a, b in customer_locations if y <= border_position]
    manhattan_locations = [(x, y) for x, y, a, b in customer_locations if y > border_position]

    best_median_row = None
    best_median_column = None
    best_total_cost = math.inf

    possible_positions = np.linspace(22.5726, 22.6141, num=100)

    # Iterate over possible distinct positions on the border
    for i in possible_positions:

        projected_euclidean = [(i, border_position) for _, _ in euclidean_locations]

        combined_customers = projected_euclidean + manhattan_locations

        median_row, median_column = calculate_manhattan_median(combined_customers)

        total_cost = sum(math.sqrt((x-i)**2 + (y-border_position)**2) for x,y in euclidean_locations) #cost from euclidian to border
        total_cost += sum(abs(x-median_row) + abs(y-median_column) for x,y in combined_customers) #cost from border to manhattan

        # total_cost = sum(math.sqrt((x - median_row)**2 + (y - median_column)**2) for x, y in combined_customers)

        # Update the best result if the current position is better
        if total_cost < best_total_cost:
            best_total_cost = total_cost
            best_median_row = median_row
            best_median_column = median_column

    return best_median_row, best_median_column

class Drone(core.Entity):
    def __init__(self):
        super().__init__()

        stack.stack(f'BOX Eucledian 22.5726 88.4010 22.6141 88.4354')
        stack.stack(f'BOX Manhattan 22.5726 88.4354 22.6141 88.4654')

        self.delevary_loc = []
        self.border = 88.4354
        del_num = 0
        for _ in range(5):
            del_num += 1
            lat = 22 + randint(5726, 6141) / 10000
            lon = 88 + randint(4010, 4654) / 10000
            if(lon > self.border):
                self.delevary_loc.append((lat, lon, del_num, 'Manhatten'))
            else:
                self.delevary_loc.append((lat, lon, del_num, 'Euclidian'))

        with self.settrafarrays():
            self.route = []
            self.x_cord = 0.0
            self.y_cord = 0.0
            # self.delevary_loc = [(22.597, 88.4503, 1, "Manhatten"),
            #                      (22.6129, 88.4209, 2, "Euclidian"),
            #                      (22.6032, 88.4417, 3, "Manhatten"),
            #                      (22.5946, 88.4566, 4, "Manhatten"),
            #                      (22.5846, 88.4105, 5, "Euclidian")]
            self.npassengers = np.array([])

    def create(self, n=1):
        super().create(n)
        self.npassengers[-n:] = [randint(0, 150) for _ in range(n)]

    # @core.timed_function(name='drone', dt=5)
    # def update(self):
    #     stack.stack(f'ECHO Total number of flights: {len(traf.id)} ')
    #     stack.stack(f'ECHO Total delevary locations : {self.delevary_loc} ')

    @stack.command
    def create_drone(self, acid: str, lat:float, lon:float):
        stack.stack(f'CRE {acid} A225 {lat} {lon} 130 FL410 485')

    @stack.command
    def create_region(self, lat1, lon1, lat2, lon2, border):
        stack.stack(f'BOX Eucledian {lat1} {lon1} {lat2} {border}')
        stack.stack(f'BOX Manhattan {lat1} {border} {lat2} {lon2}')
        self.border = border

    @stack.command
    def add_delivary(self, list_of_locations: list):
        for loc in list_of_locations:
            stack.stack(f'ECHO {loc}')

    @stack.command
    def map_delivery_locations(self):
        for loc in self.delevary_loc:
            if loc != None:
                stack.stack(f'DEFWPT TARGET-{loc[2]} {loc[0]} {loc[1]}')

    @stack.command
    def map_dp(self):
        euclidian_count = len([loc for loc in self.delevary_loc if loc[1] <= self.border])
        manhatten_count = len([loc for loc in self.delevary_loc if loc[1] > self.border])

        stack.stack(f'ECHO Total number of delivary locations in euclidian region: {euclidian_count}')
        stack.stack(f'ECHO Total number of delivary locations in manhatten region: {manhatten_count}')

        if euclidian_count >= manhatten_count:
            if manhatten_count == 0:
                stack.stack(f'ECHO All delivary locations in Euclidian region')
                self.x_cord, self.y_cord = gec(self.delevary_loc)
            else:
                stack.stack(f'ECHO Euclidian region is the most populated region')
                self.x_cord, self.y_cord = ecmb(self.delevary_loc, self.border)
        else:
            if euclidian_count == 0:
                stack.stack(f'ECHO All delivary locations in Manhatten region')
                self.x_cord, self.y_cord = gmm(self.delevary_loc)
            else:
                stack.stack(f'ECHO Manhatten region is the most populated region')
                self.x_cord, self.y_cord = mmeb(self.delevary_loc, self.border)

        acid = "drone"
        stack.stack(f'ECHO dp create at {self.x_cord} , {self.y_cord}')
        # A225 FL410 485
        stack.stack(f'CRE {acid} B105 {self.x_cord} {self.y_cord} 0 10 0')
        # create waypoint
        stack.stack(f'DEFWPT DP {self.x_cord} {self.y_cord}')
    
    @stack.command
    def create_route(self):
        distances = [(loc, math.sqrt((self.x_cord - loc[0]) ** 2 + (self.y_cord - loc[1]) ** 2)) for loc in self.delevary_loc if loc!=None]

        sorted_locations = sorted(distances, key=lambda x: x[1])

        self.route = [loc[0] for loc in sorted_locations]

        stack.stack(f'ECHO Route : {self.route}')

    @stack.command
    def start_delivery(self):
        stack.stack(f'CRELOG MYLOG 1.0')
        stack.stack(f'MYLOG ADD traf.id traf.distflown')
        stack.stack(f'MYLOG ON')

        turn_no = 0

        stack.stack(f'ADDWPT drone DP 10 10')
        stack.stack(f'POS drone')
        # this loop is ok when dp in euclidian region
        for loc in self.route:
            stack.stack(f'ECHO Navigating to {loc[0]} {loc[1]}')

            if loc[3] == "Manhatten" or self.y_cord > self.border:
                if self.y_cord < self.border and loc[3] == "Manhatten":
                    turn_no += 1
                    next_point = (loc[0],self.border)

                elif self.y_cord > self.border and loc[3] == "Euclidian":
                    turn_no += 1
                    next_point = (self.x_cord, self.border)
            
                elif self.y_cord > self.border and loc[3] == "Manhatten":
                    turn_no +=1
                    next_point = (self.x_cord, loc[1])
                    if (next_point[0] == self.x_cord and next_point[1] == self.y_cord) or (next_point[0] == loc[0] and next_point[1] == loc[1]):
                        next_point = None
            
                if next_point != None:
                    stack.stack(f'DEFWPT TURN-{turn_no} {next_point[0]} {next_point[1]}')
                    stack.stack(f'ADDWPTMODE drone FLYOVER')
                    stack.stack(f'ADDWPT drone TURN-{turn_no} 0 0')
                stack.stack(f'ADDWPTMODE drone FLYOVER')
                stack.stack(f'ADDWPT drone TARGET-{loc[2]} 0 0')
                if next_point != None:
                    stack.stack(f'ADDWPTMODE drone FLYOVER')
                    stack.stack(f'ADDWPT drone TURN-{turn_no} 0 0')

            else:
                stack.stack(f'ADDWPTMODE drone FLYOVER')
                stack.stack(f'ADDWPT drone TARGET-{loc[2]} 0 0')

            # test (capacity = 1)
            stack.stack(f'ADDWPTMODE drone FLYOVER')
            #check if it is the last location
            if loc == self.route[-1]:
                stack.stack(f'DEST drone DP')
            else:
                stack.stack(f'ADDWPT drone DP 10 10')

            # coloring
            stack.stack(f'COLOR drone yellow')

        stack.stack(f'LNAV drone ON')
        stack.stack(f'VNAV drone ON')
        # stack.stack(f'drone AT DRONE002 STACK DELWPT drone DRONE002')

    @stack.command
    def distance_covered(self, acid:'acid'):
        stack.stack(f'ECHO Distance_covers: {traf.distflown/1000} KM')

    @stack.command
    def passengers(self, acid: 'acid', count: int = -1):
        ''' Set the number of passengers on aircraft 'acid' to 'count'. '''
        if count < 0:
            return True, f'Aircraft {traf.id[acid]} currently has {self.npassengers[acid]} passengers on board.'

        self.npassengers[acid] = count
        return True, f'The number of passengers on board {traf.id[acid]} is set to {count}.'
