import math
import numpy as np

## This module enables the user to make obstacles in a pyroomacoustics environment
# The obstacles that can be generated are either as N-sided polygon or N-sided pyramid 

def polygon(centre,height,radius,N=3,thetha=0):
    # Function takes in centre as numpy array of 3 coordinates, height & radius as a Float, 
    # N is the number of sides in Integer and thetha is the orientation of the polygon

    # Function outputs a 2-D walls array

    N = int(N) # Typecasting N to integer

    # N < 3 isn't a valid case
    if N < 3:
        return

    lower_points = []
    upper_points = []
    walls = []

    z_upper = height + centre[2] # upper face z height
    z_lower = centre[2] # lower face z height
    for n in range(N):
        x = radius * math.cos((2*math.pi*n)/N + thetha) + centre[0] # generating x coordinates of all vertices
        y = radius * math.sin((2*math.pi*n)/N + thetha) + centre[1] # generating y coordinates of all vertices

        lower_points.append(np.array([x,y,z_lower])) # all lower vertices list
        upper_points.append(np.array([x,y,z_upper])) # all upper vertices list

    for i in range(N-1):
        walls.append(np.array([lower_points[i],upper_points[i],upper_points[i+1],lower_points[i+1]]))
    walls.append(np.array([lower_points[N-1],upper_points[N-1],upper_points[0],lower_points[0]])) # last case side wall from vertice N to 1st vertice
    walls.append(np.array(lower_points)) # lower face wall
    walls.append(np.array(upper_points)) # upper face wall

    return walls

def pyramid(centre,height,radius,N=3,thetha=0):
    # Function takes in centre as numpy array of 3 coordinates, height & radius as a Float, 
    # N is the number of sides in Integer and thetha is the orientation of the pyramid
    # height can be negative for upside-down orientation of pyramid

    # Function outputs a 2-D walls array

    N = int(N) # Typecasting N to integer
    
    # N < 3 isn't a valid case
    if N < 3:
        return

    lower_points = []
    walls = []

    z_upper = height + centre[2] # vertice z height
    z_lower = centre[2] # base face z height
    apex = np.array([centre[0],centre[1],z_upper]) # Apex point
    for n in range(N):
        x = radius * math.cos((2*math.pi*n)/N + thetha) + centre[0] # generating x coordinates of all vertices
        y = radius * math.sin((2*math.pi*n)/N + thetha) + centre[1] # generating y coordinates of all vertices

        lower_points.append(np.array([x,y,z_lower])) # all lower vertices list
        
    for i in range(N-1):
        walls.append(np.array([lower_points[i],lower_points[i+1],apex]))
    walls.append(np.array([lower_points[N-1],lower_points[0],apex])) # last case side wall from vertice N to 1st vertice
    walls.append(np.array(lower_points)) # lower base wall

    return walls