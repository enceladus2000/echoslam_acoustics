import matplotlib.pyplot as plt 
import numpy as np
import pyroomacoustics as pra 

material = pra.Material(energy_absorption=0.2, scattering=0.1)

def box_make(lfd,dim):
	#give wall dimensions & Left-Front-Down coordinates
	
	rfd = np.array([lfd[0]+dim[0], lfd[1], lfd[2]])
	rfu = np.array([lfd[0]+dim[0], lfd[1], lfd[2]+dim[2]])
	lfu = np.array([lfd[0], lfd[1], lfd[2]+dim[2]])
	lbd = np.array([lfd[0], lfd[1]+dim[1], lfd[2]])
	rbd = np.array([lfd[0]+dim[0], lfd[1]+dim[1], lfd[2]])
	rbu = np.array([lfd[0]+dim[0], lfd[1]+dim[1], lfd[2]+dim[2]])
	lbu = np.array([lfd[0], lfd[1]+dim[1], lfd[2]+dim[2]])
	
	walls = []

	walls.append(np.array([lfd,rfd,rfu,lfu])) #front wall
	walls.append(np.array([rfd,rbd,rbu,rfu])) #right wall
	walls.append(np.array([lfd,lbd,lbu,lfu])) #left wall
	walls.append(np.array([lbd,rbd,rbu,lbu])) #back wall
	walls.append(np.array([lfu,rfu,rbu,lbu])) #up wall
	walls.append(np.array([lfd,rfd,rbd,lbd])) #down wall

	return walls

#w1 = np.array([
	#[6., 1, 0],
	#[6., 4., 0],
	#[5., 4., 4.],
	#[5., 1, 4.]
#])

room_dim = [10, 10, 5]
box = pra.ShoeBox(room_dim, fs=16000, materials=material)
# box2 = pra.ShoeBox([1, 3, 4], fs=16000, materials=material)
walls = box.walls

corner = np.array([4,4,0])
dim = np.array([5,5,3])
obstacle_faces = box_make(corner,dim)

for face in obstacle_faces:
	walls.append(pra.wall_factory(face.T, material.energy_absorption["coeffs"], material.scattering["coeffs"]))

room = pra.Room(walls, fs=16000, max_order=3, ray_tracing=True)
room.add_source([3, 6, 1.])
room.add_microphone([3, 3, 1.])

print(room.walls)
room.image_source_model()

room.plot(img_order=1)
plt.show()