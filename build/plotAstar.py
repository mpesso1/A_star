import matplotlib
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
#plt.style.use('classic')


data = np.loadtxt("/home/mason/Code/Astar/build/storeAstar.txt",dtype=float)
dataPath = np.loadtxt("/home/mason/Code/Astar/build/storePath.txt",dtype=float)

def traj(): # function for plotting out multiple trajectories and objects from shell.
            # Possile issues:   conditional for if statement and how data is coming in from cpp -gpmp.cpp
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    
    x = np.zeros(np.shape(data)[0])
    y = np.zeros(np.shape(data)[0])
    z = np.zeros(np.shape(data)[0])

    #print(np.shape(x))

    for i in range(np.shape(data)[0]-1):
        #ax.scatter(data[i,0],data[i,1],data[i,2],'yellow',s=5) # use to plot objects
        x[i] = data[i,0] 
        y[i] = data[i,1]
        z[i] = data[i,2]

    
    x_path = np.zeros(np.shape(dataPath)[0])
    y_path = np.zeros(np.shape(dataPath)[0])
    z_path = np.zeros(np.shape(dataPath)[0])

    #print(np.shape(x))

    for j in range(np.shape(dataPath)[0]-1):
        ax.scatter(dataPath[j,0],dataPath[j,1],dataPath[j,2],'yellow',s=5) # use to plot objects
        x_path[j] = dataPath[j,0] 
        y_path[j] = dataPath[j,1]
        z_path[j] = dataPath[j,2]
    
    ax.axes.set_xlim3d(left=-0, right=12) 
    ax.axes.set_ylim3d(bottom=-0, top=10) 
    ax.axes.set_zlim3d(bottom=-0, top=12) 

    ax.plot3D(x,y,z,label="hey")
    #ax.plot3D(x_path,y_path,z_path,label="hey")
    #print(x)
    plt.show()

traj()