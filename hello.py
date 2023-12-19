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

    return centroid[0], centroid[1]

def calculate_manhattan_median(customers):
    # Extract coordinates of customer locations
    coordinates = [(x, y) for x, y in customers]

    # Calculate median coordinates for rows and columns
    median_row = sorted([x for x, y in coordinates])[len(coordinates) // 2]
    median_column = sorted([y for x, y in coordinates])[len(coordinates) // 2]

    return median_row, median_column

def mmepb_algorithm(customer_locations, border_position):
    R = border_position  # Number of distinct positions on the border

    # Separate customer locations into Euclidean and Manhattan sides
    euclidean_locations = [(x, y) for x, y, a, b in customer_locations if y <= border_position]
    manhattan_locations = [(x, y) for x, y, a, b in customer_locations if y > border_position]

    # Initialize variables to store the best result
    best_median_row = None
    best_median_column = None
    best_total_cost = math.inf

    # Generate a range of float values for R
    # possible_positions = np.linspace(1, R, num=100)  # Adjust num as needed
    possible_positions = np.linspace(22.5726, 22.6141, num=100)

    # Iterate over possible distinct positions on the border
    for i in possible_positions:
        # Project Euclidean customers on the border
        projected_euclidean = [(i, border_position) for _, _ in euclidean_locations]
        print(projected_euclidean)

        # Combine projected Euclidean and Manhattan customers
        combined_customers = projected_euclidean + manhattan_locations

        # Calculate median coordinates for rows and columns
        median_row, median_column = calculate_manhattan_median(combined_customers)

        # Calculate the total cost for the current position on the border
        total_cost = sum(math.sqrt((x - median_row)**2 + (y - median_column)**2) for x, y in combined_customers)

        # Update the best result if the current position is better
        if total_cost < best_total_cost:
            best_total_cost = total_cost
            best_median_row = median_row
            best_median_column = median_column

    return best_median_row, best_median_column

# Example usage:
customer_locations = [(22.597, 88.4503, 1, 2),
                     (22.6129, 88.4209, 2, 3),
                     (22.6032, 88.4417, 3, 4),
                     (22.5946, 88.4566, 4, 5),
                     (22.5846, 88.4105, 5, 6)]
border_position = 88.4354
dp_coordinates = mmepb_algorithm(customer_locations, border_position)
print("Distribution Point (DP) coordinates:", dp_coordinates)
print(gmm(customer_locations))
