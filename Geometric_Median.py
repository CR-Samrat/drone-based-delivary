import numpy as np

def geometric_median(delivary_points,border=None, eps=1e-5):
    # Initial guess for the geometric median
    points = []
    if border != None:
        for point in delivary_points:
            if point[0] < border:
                points.append(point)
            else:
                points.append([border, point[1]])
    else:
        points = delivary_points.copy()

    guess = np.mean(points, axis=0)
    # guess = np.array(initial_guess)
    
    while True:
        # Calculate distances from current guess to all points
        distances = np.linalg.norm(points - guess, axis=1)
        
        # Avoid division by zero
        if np.sum(distances) == 0:
            return guess
        
        # Calculate weights as inverse distances
        weights = 1 / distances
        
        # Calculate weighted average
        new_guess = np.sum(points * weights[:, np.newaxis], axis=0) / np.sum(weights)
        
        # Check for convergence
        if np.linalg.norm(new_guess - guess) < eps:
            return new_guess
        
        guess = new_guess