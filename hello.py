def ecmb(coordinate):
    euclidean_customer = [loc for loc in coordinate if loc[1] >= 74 and loc[1] <= 79.99]
    manhattan_customer = [loc for loc in coordinate if loc[1] >= 80 and loc[1] <= 83.00]

    x_axis = sum(customer[0] for customer in coordinate) / len(coordinate)
    border = max(customer[0] for customer in euclidean_customer)

    y_axis = (sum(customer[1] for customer in euclidean_customer) + (len(manhattan_customer) * border)) / len(coordinate)

    final_centroid = [x_axis, y_axis]

    return x_axis, y_axis

if __name__ == '__main__':
    delivary_loc = [(21.4, 75.7),(23.5, 78.4),(22.3, 79.2),(21.0, 82.8),(22.2, 82.9)]
    x_cord, y_cord = ecmb(delivary_loc)

    print(x_cord, y_cord)