import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra
import math

material = pra.Material(energy_absorption=0.2, scattering=0.1)

def polygon_make(centre,height,radius,N=3,thetha=0):
    #Function takes in centre as numpy array of 3 coordinates, height & radius as a float, 
    #N is the number of sides in integer and thetha is the orientation of the polygon

    #Function outputs a 2-D walls array

    lower_points = []
    upper_points = []
    walls = []

    z_upper = height + centre[2] #upper face z height
    z_lower = centre[2] #lower face z height
    for n in range(N):
        x = radius * math.cos((2*math.pi*n)/N + thetha) + centre[0] #generating x coordinates of all vertices
        y = radius * math.sin((2*math.pi*n)/N + thetha) + centre[1] #generating y coordinates of all vertices

        lower_points.append(np.array([x,y,z_lower])) #all lower vertices list
        upper_points.append(np.array([x,y,z_upper])) #all upper vertices list

    for i in range(N-1):
        walls.append(np.array([lower_points[i],upper_points[i],upper_points[i+1],lower_points[i+1]]))
    walls.append(np.array([lower_points[N-1],upper_points[N-1],upper_points[0],lower_points[0]])) #last case side wall from vertice N to 1st vertice
    walls.append(np.array(lower_points)) #lower wall
    walls.append(np.array(upper_points)) #upper wall

    return walls

#Code test example
room_dim = [10, 10, 5]
box = pra.ShoeBox(room_dim, fs=16000, materials=material)
walls = box.walls

centre = np.array([4,4,0])
height = 3
radius = 2
N = 3
#thetha = math.pi/4
obstacle_faces = polygon_make(centre,height,radius,N)

for face in obstacle_faces:
	walls.append(pra.wall_factory(face.T, material.energy_absorption["coeffs"], material.scattering["coeffs"]))

room = pra.Room(walls, fs=16000, max_order=3, ray_tracing=True)
room.add_source([3, 6, 1.])
room.add_microphone([3, 3, 1.])

print(room.walls)
room.image_source_model()

room.plot(img_order=1)
plt.show()