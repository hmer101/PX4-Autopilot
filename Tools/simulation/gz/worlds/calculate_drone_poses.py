import numpy as np

# Also see: multi_drone_slung_load's generate_spawn_points.py (does yaws too)


def generate_points_circle(n_points, radius, offset_x=0.0):
    """
    Generate a list of 2D points that lie on a circle.
    
    Parameters:
    - n_points: Number of points to generate.
    - radius: Radius of the circle.
    
    Returns:
    - List of tuples representing (x, y) coordinates on the circle.
    """
    points = []
    
    for i in range(n_points):
        angle = i * 2 * np.pi / n_points
        x = radius*np.cos(angle) + np.sign(np.cos(angle))*offset_x
        y = radius*np.sin(angle)
        points.append((x, y))
    
    return points


def transform_points(points, translation, rotation_angle):
    """
    Transform a list of 2D points from one coordinate system to another.
    
    Parameters:
    - points: List of tuples representing (x, y) coordinates in the original coordinate system.
    - translation: Tuple representing (tx, ty) translation vector.
    - rotation_angle: Angle in degrees by which to rotate the coordinate system.
    
    Returns:
    - Transformed points in the new coordinate system as a list of tuples.
    """
    # Convert rotation angle from degrees to radians
    #theta = np.deg2rad(rotation_angle)
    theta = rotation_angle
    
    # Define the rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    
    transformed_points = []
    
    for x, y in points:
        # Rotate the point
        rotated_point = rotation_matrix @ np.array([x, y])
        
        # Translate the rotated point
        translated_point = rotated_point + np.array(translation)
        
        # Append the result
        transformed_points.append(tuple(translated_point))
    
    return transformed_points


## DEFINE PARAMETERS
# Define load pose
translation = (0, 0)
rotation_angle = 0

# Drones rel load
n_points = 3
r_drones = 1.645 #1.645 (2.4171 m cable) 1.635 (2.40147 m cable) 1.65 (2.425 m cable) 1.625 (2.3858 m cable) 1.621 (2.37957 m cable) 2.1 (3.13 m cable) #1.5 (2.19 m cable) #m

# Tether placement
r_tethers = 0.1 #m

d_tether_from_drone_com_x = 0.04

## ADD OFFSETS
#r_drones += d_tether_from_drone_com # Offset to account for the distance from drone COM to tether placement point

## GENERATE POINTS
points_drones = generate_points_circle(n_points, r_drones, offset_x=d_tether_from_drone_com_x) # Generate points around the load pose
transformed_points_drones = transform_points(points_drones, translation, rotation_angle) # Transform points to surround the desired load pose

# Generate tether placement points
points_tethers = generate_points_circle(n_points, r_tethers)
transformed_points_tethers = transform_points(points_tethers, translation, rotation_angle) # Transform points to surround the desired load pose


## PRINT
print("DRONES: ")
print(transformed_points_drones)

print("TETHERS: ")
print(transformed_points_tethers)
