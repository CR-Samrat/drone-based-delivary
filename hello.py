def gec(coordinates):
    x_sum = sum(loc[0] for loc in coordinates)
    y_sum = sum(loc[1] for loc in coordinates)
    x_cord = x_sum / len(coordinates)
    y_cord = y_sum / len(coordinates)

    return x_cord, y_cord

def ecmb(coordinate):
    longitude = 0
    latitude = 0
    k=88.4354
    test = gec(coordinate)
    print("Test",test)
    for loc in coordinate:
        if loc[1] <= k:
            longitude+= loc[1]
        else:
            longitude+= k
        latitude+= loc[0]
    y_axis = longitude/len(coordinate)
    x_axis = latitude/len(coordinate)
    return x_axis, y_axis

if __name__ == '__main__':
    delivary_loc = [(22.597, 88.4203),(22.6129, 88.4209),(22.6032, 88.4417),(22.5946, 88.4566),(22.5846, 88.4105)]
    x_cord, y_cord = ecmb(delivary_loc)

    print(x_cord, y_cord)