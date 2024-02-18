import random
import numpy as np
from bluesky import core, stack, traf
import math
from k_means_constrained import KMeansConstrained

def init_plugin():
    drone = Adv_drone()

    config = {
        'plugin_name':     'MULTI_CAPACITY_DELIVERY',
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

# Main program
class Adv_drone(core.Entity):
    def __init__(self):
        super().__init__()

        stack.stack(f'BOX Eucledian 22.5726 88.4010 22.6141 88.4654')

        self.n = 25
        self.k = 5
        self.x = [22.5726, 88.4010]
        self.y = [22.6141, 88.4654]

        self.delivary_loc = create_locations(self.x, self.y, self.n)
        self.centroids, self.clusters = KMeans(self.delivary_loc, self.n, self.k)
        self.dp = calculate_dp(self.centroids)

    @stack.command
    def create_drone(self, acid: str, lat:float, lon:float):
        stack.stack(f'CRE {acid} A225 {lat} {lon} 130 FL410 485')

    @stack.command
    def map_delivary_locations(self):
        idx = 0
        for loc in self.delivary_loc:
            idx+=1
            stack.stack(f'DEFWPT T-{idx} {loc[0]} {loc[1]}')

    @stack.command
    def map_dp(self):
        stack.stack(f'DEFWPT DP {self.dp[0]} {self.dp[1]}')