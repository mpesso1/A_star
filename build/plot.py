import matplotlib
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np


def traj(): # function for plotting out multiple trajectories and objects from shell.
            # Possile issues:   conditional for if statement and how data is coming in from cpp -gpmp.cpp

    ax = plt.axes(projection='3d')
    
    x = []
    xmax = 0
    xmin = 0
    y = []
    ymax = 0
    ymin = 0
    z = []
    zmax = 0
    zmin = 0


    obj_num = 1
    Traj_num = 1
    cout = 0
    doneWithTraj = False

    obs = False
    with open('/home/mason/Code/Astar/build/storePlot.txt') as f:
        ww = 0
        for line in f:
            # print(i)
            s = line.split()
            print(s)
            
            # print(s)
            if s[0] == "o":
                obs = True
                continue

            if obs == False:

                for i in range(0,3):
                    s[i] = float(s[i])
                
                if ww==0:
                    ax.scatter(s[0],s[1],s[2], color='green', s=50)
                    ww = ww+1

                # ax.scatter(s[0],s[1],s[2],'yellow',s=50)
                x.append(s[0])
                y.append(s[1])
                z.append(s[2])

            else:
                for i in range(0,3):
                    s[i] = float(s[i])
                ax.scatter(s[0],s[1],s[2],color='red',s=50)

        # ax.scatter(s[0],s[1],s[2],color='yellow',s=50)
        ax.scatter(12000,9900,12000,color='yellow',s=50)
        ax.plot3D((x),(y),(z))
        ax.set_xlabel('X')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()


traj()