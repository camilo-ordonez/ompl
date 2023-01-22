import matplotlib.pyplot as plt
import csv

X = []
Y = []

with open('~/Documents/omplplanning/myplanners/build/test.txt', 'r') as datafile:
	plotting = csv.reader(datafile, delimiter=',')
	
	for ROWS in plotting:
		X.append(int(ROWS[0]))
		Y.append(int(ROWS[1]))

plt.plot(X, Y)
plt.title('Line Graph using CSV')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

