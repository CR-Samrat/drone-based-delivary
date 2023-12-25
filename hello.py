import math
import numpy as np

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

def calculate_manhattan_median(customers):
    coordinates = [(x, y) for x, y in customers]

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

    for i in possible_positions:

        projected_euclidean = [(i, border_position) for _, _ in euclidean_locations]

        combined_customers = projected_euclidean + manhattan_locations

        median_row, median_column = calculate_manhattan_median(combined_customers)

        #try
        total_cost = sum(math.sqrt((x-i)**2 + (y-border_position)**2) for x,y in euclidean_locations) #cost from euclidian to border
        total_cost += sum(abs(x-median_row) + abs(y-median_column) for x,y in combined_customers) #cost from border to manhattan

        # total_cost = sum(math.sqrt((x - median_row)**2 + (y - median_column)**2) for x, y in combined_customers)

        print(total_cost, best_total_cost)

        if total_cost < best_total_cost:
            best_total_cost = total_cost
            best_median_row = median_row
            best_median_column = median_column

    return best_median_row, best_median_column

if __name__ == "__main__":
    delevary_loc = [(22.597, 88.4503, 1, "Manhatten"),
                          (22.6129, 88.4209, 2, "Euclidian"),
                          (22.6032, 88.4417, 3, "Manhatten"),
                          (22.5946, 88.4566, 4, "Manhatten"),
                          (22.5846, 88.4105, 5, "Euclidian")]
    border = 88.4354
    print(gmm(delevary_loc))
    print(mmeb(delevary_loc, border))