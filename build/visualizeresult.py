import matplotlib.pyplot as plt
import numpy as np

#filename_result = input("Enter the name of the planing result file. e.g., result.txt")

#filedata = np.loadtxt(filename_result, dtype = float)
filedata = np.loadtxt('result.txt', dtype = float)

x = filedata[:,0]
y = filedata[:,1]
th = filedata[:,2]


#filename_obst = input("Enter the name of the obstacle file. e.g., obstacles.txt")
#filename = "../" + filename_obst 
#print(filename)

#filename = "../obstacles_large.txt"
filename = "../obstacles.txt"

obstacle_data = np.loadtxt(filename, dtype = float)

if (obstacle_data.ndim > 1):
	xc = obstacle_data[:,0]
	yc = obstacle_data[:,1]
	radc = obstacle_data[:,2]

elif (obstacle_data.ndim == 1):
	xc = obstacle_data[0]
	yc = obstacle_data[1]
	radc = obstacle_data[2]
else:
	print("obstacle file is empty")


#plt.axis([-10, 10, -10, 10])
#plt.axis("equal")
plt.xlim([-10, 10])
plt.ylim([-10, 10])


# plot the obstalces


if (obstacle_data.ndim == 1):
	c1 = plt.Circle((xc, yc), radc)
	plt.gca().add_artist(c1)
	
	
else:
	it = 0
	for it2 in xc:
		c1 = plt.Circle((xc[it], yc[it]), radc[it])
		plt.gca().add_artist(c1)
		it = it + 1

plt.plot(x, y)
#plt.title('Line Graph using NUMPY')
plt.xlabel('x')
plt.ylabel('y')
#plt.axis("equal")
#plt.xlim([-10, 10])
#plt.ylim([-10, 10])

#axs[0, 1].axis('equal')
#axs[0, 1].set_title('equal, looks like circle', fontsize=10)

plt.axis("equal")
#plt.xlim([-10, 10])
#plt.ylim([-10, 10])

plt.show()

#plt.axis("equal")


