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

def ecmb(coordinate):
    euclidean_customer = [loc for loc in coordinate if loc[1] >= 74 and loc[1] <= 79.99]
    manhattan_customer = [loc for loc in coordinate if loc[1] >= 80 and loc[1] <= 83.00]

    x_axis = sum(customer[0] for customer in coordinate) / len(coordinate)
    border = max(customer[0] for customer in euclidean_customer)

    y_axis = (sum(customer[1] for customer in euclidean_customer) + (len(manhattan_customer) * border)) / len(coordinate)

    final_centroid = [x_axis, y_axis]

    return x_axis, y_axis

class Drone(core.Entity):
    def __init__(self):
        super().__init__()

        stack.stack('BOX Eucledian 22.5726 88.4010 22.6141 88.4354')
        stack.stack('BOX Manhattan 22.5726 88.4354 22.6141 88.4654')

        self.delevary_loc = []
        for _ in range(5):
            lat = 22 + randint(5726, 6141) / 10000
            lon = 88 + randint(4010, 4654) / 10000
            self.delevary_loc.append((lat, lon))

        with self.settrafarrays():
            self.route = []
            self.x_cord = 0.0
            self.y_cord = 0.0
            # self.delevary_loc = [(20.51, 77.12),(21.73, 77.91),(22.98, 76.53),(21.11, 76.12),(20.10, 76.11)]
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
    def find_dp(self):
        euclidian_count = len([loc for loc in self.delevary_loc if loc[1] >= 88.4010 and loc[1] <= 88.4354])
        manhatten_count = len([loc for loc in self.delevary_loc if loc[1] > 88.4354 and loc[1] <= 88.4654])

        stack.stack(f'ECHO Total number of delivary locations in euclidian region: {euclidian_count}')
        stack.stack(f'ECHO Total number of delivary locations in manhatten region: {manhatten_count}')

        if euclidian_count >= manhatten_count:
            if manhatten_count == 0:
                stack.stack(f'ECHO All delivary locations in Euclidian region')
                self.x_cord, self.y_cord = gec(self.delevary_loc)
            else:
                stack.stack(f'ECHO Euclidian region is the most populated region')
                self.x_cord, self.y_cord = gec(self.delevary_loc)
        else:
            stack.stack(f'ECHO Manhatten region is the most populated region')
            self.x_cord, self.y_cord = gmm(self.delevary_loc)

        acid = "drone"
        stack.stack(f'ECHO dp create at {self.x_cord} , {self.y_cord}')
        # A225 FL410 485
        stack.stack(f'CRE {acid} B105 {self.x_cord} {self.y_cord} 0 10 0')
    
    @stack.command
    def find_route(self):
        distances = [(loc, math.sqrt((self.x_cord - loc[0]) ** 2 + (self.y_cord - loc[1]) ** 2)) for loc in self.delevary_loc]

        sorted_locations = sorted(distances, key=lambda x: x[1])

        self.route = [loc[0] for loc in sorted_locations]

        stack.stack(f'ECHO Route : {self.route}')

    @stack.command
    def start_delivary(self):
        stack.stack(f'ADDWPT drone {self.x_cord} {self.y_cord} 10 10')
        stack.stack(f'POS drone')
        for loc in self.route:
            stack.stack(f'ECHO Navigating to {loc[0]} {loc[1]}')
            stack.stack(f'ADDWPTMODE drone FLYOVER')
            stack.stack(f'ADDWPT drone {loc[0]} {loc[1]} 0 0')
            # test (capacity = 1)
            stack.stack(f'ADDWPTMODE drone FLYOVER')
            #check if it is the last location
            if loc == self.route[-1]:
                stack.stack(f'DEST drone {self.x_cord} {self.y_cord}')
            else:
                stack.stack(f'ADDWPT drone {self.x_cord} {self.y_cord} 10 10')

        stack.stack(f'LNAV drone ON')
        stack.stack(f'VNAV drone ON')
        # stack.stack(f'drone AT DRONE002 STACK DELWPT drone DRONE002')

    @stack.command
    def passengers(self, acid: 'acid', count: int = -1):
        ''' Set the number of passengers on aircraft 'acid' to 'count'. '''
        if count < 0:
            return True, f'Aircraft {traf.id[acid]} currently has {self.npassengers[acid]} passengers on board.'

        self.npassengers[acid] = count
        return True, f'The number of passengers on board {traf.id[acid]} is set to {count}.'
