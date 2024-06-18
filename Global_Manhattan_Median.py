import numpy as np
import math

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

def mmeb(customer_locations, border_position):
    R = border_position  

    euclidean_locations = [(x, y) for x, y in customer_locations if x < border_position]
    manhattan_locations = [(x, y) for x, y in customer_locations if x >= border_position]

    best_median_row = None
    best_median_column = None
    best_projected_euclidean = None
    best_total_cost = math.inf

    possible_positions = np.linspace(88.4010, 88.4654, num=10)

    # Iterate over possible distinct positions on the border
    for i in possible_positions:

        projected_euclidean = [(border_position, i) for _, _ in euclidean_locations]

        combined_customers = projected_euclidean + manhattan_locations

        median_row, median_column = gmm(combined_customers)

        total_cost = sum(math.sqrt((x-border_position)**2 + (y-i)**2) for x,y in euclidean_locations) #cost from euclidian to border
        total_cost += sum(abs(x-median_row) + abs(y-median_column) for x,y in combined_customers) #cost from border to manhattan

        # Update the best result if the current position is better
        if total_cost < best_total_cost:
            best_projected_euclidean = projected_euclidean[0]
            best_total_cost = total_cost
            best_median_row = median_row
            best_median_column = median_column

    return [best_median_row, best_median_column], best_projected_euclidean