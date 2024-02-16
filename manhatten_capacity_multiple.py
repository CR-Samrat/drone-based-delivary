from random import randint
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
from k_means_constrained import KMeansConstrained

delevary_loc = []
n = 50

#tsp = crystofice
def find_dp(centroids):
    print("Centroid",centroids)
    x = [loc_x for loc_x, loc_y in centroids]
    y = [loc_y for loc_x, loc_y in centroids]
    
    mean_x = np.median(x)
    mean_y = np.median(y)
    return (mean_x, mean_y)

def calculate_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# incomplete
def tsp(cluster, opt_dp):
    cluster.insert(0, opt_dp)
    unvisited = cluster.copy()
    tour = [opt_dp]
    unvisited.remove(opt_dp)

    while unvisited:
        nearest_point = min(unvisited, key=lambda x: calculate_distance(tour[-1], x))
        tour.append(nearest_point)
        unvisited.remove(nearest_point)

    # Return to the starting point to complete the cycle
    tour.append(opt_dp)

    print("Route:",tour)
    return tour

def build_cluster(no_of_clusters):

    # KMeans implementation
    clf = KMeansConstrained(n_clusters=no_of_clusters, size_min=1, size_max=5)
    x = np.array(delevary_loc)
    labels = clf.fit_predict(x)
    centroids = clf.cluster_centers_

    #creating a mapping
    mapping = {label: [] for label in range(no_of_clusters)}
    for i in range(len(delevary_loc)):
        mapping[labels[i]].append(delevary_loc[i])

    return mapping, labels, centroids, x

def plot_data(labels, centroids, x, opt_dp, optimal_path):
    colors = ['blue', 'green', 'orange', 'purple', 'brown']
    plt.scatter(x[:, 0], x[:, 1], c=labels, cmap='viridis')
    plt.scatter(centroids[:, 0], centroids[:, 1], marker='X', s=200, c='red')
    plt.scatter(opt_dp[0], opt_dp[1], marker='^')
    for path in optimal_path:
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]

        # Plotting lines with only horizontal and vertical segments
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            color = colors[randint(0, len(colors) - 1)]
            # Connect points with horizontal and vertical segments
            plt.plot([x1, x2], [y1, y1], linestyle='-', linewidth=1, color=color)
            plt.plot([x2, x2], [y1, y2], linestyle='-', linewidth=1, color=color)
    plt.title(f'K-means Clustering with no of clusters = {no_of_clusters}')
    plt.show()

def find_del_loc():
    for _ in range(n):
        lat = 22 + randint(5726, 6141) / 10000
        lon = 88 + randint(4010, 4654) / 10000
        loc = (lat, lon)
        delevary_loc.append(loc)
    return delevary_loc

if __name__ == "__main__":

    # creating the boundary
    x1 = 22.5726
    y1 = 88.4010
    x2 = 22.6141
    y2 = 88.4654
    print("Delivary locations",find_del_loc())

    # calculating no of clusters
    no_of_clusters = n//5
    if(no_of_clusters*5 < n):
        no_of_clusters += 1

    # getting output from k means in the from of a mapping
    cluster_ponts, labels, centroids, x = build_cluster(no_of_clusters)
    
    #finding dp from centroids
    opt_dp = find_dp(centroids)
    print("Dp ",opt_dp)

    # applying tsp algorithm on each cluster
    optimal_path = []
    for i in range(len(cluster_ponts)):
        path = tsp(cluster_ponts[i], opt_dp)
        optimal_path.append(path)

    #plotting the data
    plot_data(labels, centroids, x, opt_dp, optimal_path)