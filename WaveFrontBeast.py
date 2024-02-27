# -*- coding: utf-8 -*-
"""
Created on Tue Oct  4 21:12:59 2022

@author: riana
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Oct  3 22:44:59 2022

@author: riana
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Oct  3 16:13:47 2022

@author: riana

"""

#initialize the obstacles in the workspace as 1
def intialize(O,xv,yv,x_store,y_store): 
    
    #input : Obstacle [x,y] (in counter clockwise order, include first vert twice)
    #input : x [x,y] coordinate of bug on current path
    

    if O.ndim == 2:
       numVert = len(O)
       
       for k in range(len(xv)):
           for j in range(len(yv)):
               a = np.array([])
               for i in range(numVert-1):
           
                   H = -((O[i,1]- O[i+1,1])*xv[k][j]+(O[i+1,0]- O[i,0])*yv[k][j] + (O[i,0]*O[i+1,1]- O[i+1,0]*O[i,1]))
        
                   if H < 0:
                       a = np.append(a,1)  
               
                   if H > 0 :
                        a = np.append(a,0)
                        break
                    
               if a.all() == True :
                    y_store[k][j] = 1
                    x_store[k][j] = 1
  
           
    if O.ndim == 3:
        
        numOb= len(O)
        for i in range(numOb):
            numVert = len(O[i])
            for k in range(len(xv)):
                for l in range(len(yv)):
                    a = np.array([])
                    
                    for j in range(numVert-1):
          
                        H = -((O[i][j,1]- O[i][j+1,1])*xv[k][l]+(O[i][j+1,0]- O[i][j,0])*yv[k][l] + (O[i][j,0]*O[i][j+1,1]- O[i][j+1,0]*O[i][j,1]))
       
                        if H < 0:
                            a = np.append(a,1)  
              
                        if H > 0 :
                            a = np.append(a,0)
                            break
                   
                    if a.all() == True :
                        y_store[k][l] = 1
                        x_store[k][l] = 1

                
          
    return x_store,y_store

#complete the wavefront planner for the whole workspace
def TheWave(xv,yv,x_store,y_store,step):
    
    counter = 2
    
    while x_store.all() == False and y_store.all() == False: 
        
        for i in range(len(xv)):
            for j in range(len(yv)):
                if x_store[i][j] == counter and y_store[i][j] == counter:
                       
                    if xv[i][j] + step in xv:
                         p = np.argwhere(xv == xv[i][j] + step)
                         q = np.argwhere(yv == yv[i][j])
                         ly = p[2,1]
                         lx = q[2,0]
                             
                         if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                             x_store[lx,ly] = counter + 1
                             y_store[lx,ly] = counter + 1
                         else:
                             x_store[lx,ly] = x_store[lx,ly]
                             y_store[lx,ly]= y_store[lx,ly]
                       
                         
                    if xv[i][j] - step in xv:
                        p = np.argwhere(xv == xv[i][j] - step)
                        q = np.argwhere(yv == yv[i][j])
                        ly = p[2,1]
                        lx = q[2,0]
                             
                        if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                            x_store[lx,ly] = counter + 1
                            y_store[lx,ly] = counter + 1
                        else:
                            x_store[lx,ly] = x_store[lx,ly]
                            y_store[lx,ly]= y_store[lx,ly]
                                 
                    if yv[i][j] + step in yv:
                         p = np.argwhere(xv == xv[i][j])
                         q = np.argwhere(yv == yv[i][j] + step)
                         ly = p[2,1]
                         lx = q[2,0]
                             
                         if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                             x_store[lx,ly] = counter + 1
                             y_store[lx,ly] = counter + 1
                         else:
                             x_store[lx,ly] = x_store[lx,ly]
                             y_store[lx,ly]= y_store[lx,ly]
                         
                    if yv[i][j] - step in yv:
                         p = np.argwhere(xv == xv[i][j])
                         q = np.argwhere(yv == yv[i][j] - step)
                         ly = p[2,1]
                         lx = q[2,0]
                             
                         if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                             x_store[lx,ly] = counter + 1
                             y_store[lx,ly] = counter + 1
                         else:
                             x_store[lx,ly] = x_store[lx,ly]
                             y_store[lx,ly]= y_store[lx,ly]
                        
            
        counter += 1
        
    
    
    return x_store,y_store
                
           
#search for the optimal path

            


    
    

import numpy as np
import matplotlib.pyplot as plt
import math
import random
step = .25

#created a mesh grid of points between 0 and .25
nx, ny = (173,173)
x = np.linspace(-7, 36, nx)
y = np.linspace(-7, 36, ny)
xv, yv = np.meshgrid(x, y)

#store the values of the wavefront in here that correspond to the point on meshgrid
x_store = xv*0
y_store= yv*0

#initial values
q_start = [0,0]
q_goal = [35,0]

#initialize the goal position points as 2 in our 
for i in range(len(xv)):
    for j in range(len(yv)):
        if xv[i][j] == q_goal[0] and yv[i][j] == q_goal[1]:
            x_store[i][j] = 2
            y_store[i][j] = 2
            

#np.argwhere will be important

#obstacle 1
#two square obstacles
#OBSTACLE 1 centered at (4,1) side length 1
v11 = [-6,-6] 
v12 = [25,-6]
v13 = [25,-5]
v14 = [-6,-5]

vert_W01 = np.array([v11,v12,v13,v14,v11])

v21 = [-6,5]
v22 = [30,5]
v23 = [30,6]
v24 = [-6,6]

vert_W02 = np.array([v21,v22,v23,v24,v21])

#WO3b 
v31 = [-6,-5]
v32 = [-5,-5]
v33 = [-5,5]
v34 = [-6,5]

vert_W03 = np.array([v31,v32,v33,v34,v31])

#W04b
v41 = [4,-5]
v42 = [5,-5]
v43 = [5,1]
v44 = [4,1]

vert_W04 = np.array([v41,v42,v43,v44,v41])

#WO5b
v51 = [9,0]
v52 = [10,0]
v53 = [10,5]
v54 = [9,5]

vert_W05 = np.array([v51,v52,v53,v54,v51])

v61 = [14,-5]
v62 = [15,-5]
v63 = [15,1]
v64 = [14,1]

vert_W06 = np.array([v61,v62,v63,v64,v61])

v71 = [19,0]
v72 = [20,0]
v73 = [20,5]
v74 = [19,5]

vert_W07 = np.array([v71,v72,v73,v74,v71])

v81 = [24,-5]
v82 = [25,-5]
v83 = [25,1]
v84 = [24,1]

vert_W08 = np.array([v81,v82,v83,v84,v81])

v91 = [29,0]
v92 = [30,0]
v93 = [30,5]
v94 = [29,5]

vert_W09 = np.array([v91,v92,v93,v94,v91])

v1 = [3.5,0.5]
v2 = [4.5,0.5]
v3 = [4.5,1.5]
v4 = [3.5,1.5]

#create the obstacle, the beast
o1 = np.array([vert_W01,vert_W02,vert_W03,vert_W04,vert_W05,vert_W06, vert_W07,vert_W08,vert_W09])


#x_store and y_store now know which points correspond to points in an obstacle
x_store,y_store = intialize(o1, xv, yv, x_store, y_store)

#the completed wavefront, x_store and y_store should be identical
x_store,y_store = TheWave(xv,yv,x_store,y_store,step)

#The path it decides to take, store values 
path = np.array([])

p_start = np.argwhere(xv == 0)
q_start = np.argwhere(yv == 0)
ly_start= p_start[2,1] #rnadom chosen values, the y corresponds to any y in this array
lx_start = q_start[2,0]#rnadom chosen values, the y corresponds to any x in this array

start_value = x_store[lx_start,ly_start]
q_start = [xv[lx_start,ly_start],yv[lx_start,ly_start]]
path = np.append(path,q_start)
i = 1
q = q_start
grid = start_value

while (grid - (i-1)) != 2:
    
    #have the value of goal 
    step_x = xv[x_store == grid - i]
    step_y = yv[x_store == grid - i]
    
    options_x = np.array([])
    options_y = np.array([])
    
    for j in range(len(step_x)):

        if (step_x[j] >= q[0] - step and step_x[j] <= q[0]+step) and (step_y[j] >= q[1] - step and step_y[j] <= q[1]+step) and (math.dist([step_x[j],step_y[j]],q_goal) < math.dist(q,q_goal)):
            x = step_x[j]
            options_x = np.append(options_x,x)
            y = step_y[j]
            options_y = np.append(options_y,y)
            
        if (step_x[j] >= q[0] - step and step_x[j] <= q[0]+step) and (step_y[j] >= q[1] - step and step_y[j] <= q[1]+step):
            x = step_x[j]
            options_xb = np.append(options_x,x)
            y = step_y[j]
            options_yb = np.append(options_y,y)
            
            
            
        
    if len(options_x) > 1 or len(options_y) > 1:
        ind = random.randint(0,len(options_x)-1)
        qx = options_x[ind]
        qy = options_y[ind]
    else:
        qx = options_x
        qy = options_y
        
    if qx.size == 0:
        if len(options_xb) > 1 or len(options_yb) > 1:
            ind = random.randint(0,len(options_xb)-1)
            qx = options_xb[ind]
            qy = options_yb[ind]
        else:
            qx = options_xb
            qy = options_yb
    
    #move to a point where the grid value is 1 less
    #if one less does not exist, move to point where grid value is equal
    #if more than one point exists
    
    q = [qx,qy]
    path = np.append(path, q)

    path = path.reshape(i+1,2)
    
    #lets assume for now that it doesnt run into problems where it stays on same grid number
    
    if path[i-1,0] == qx and path[i-1,1] == qy:
        i = 1
        path = np.array([])
        path = np.append(path,q_start)
        q = q_start

        
        
    elif i >= 2 and path[i-2,0] == qx and path[i-2,1] == qy:
        i = 1
        path = np.array([])
        path = np.append(path,q_start)
        q = q_start

    else:
        i += 1
        
    
    
    


#plt.axis([0, 10, -2, 2])
plt.plot(vert_W01[:,0],vert_W01[:,1],linewidth = .5)
plt.plot(vert_W02[:,0],vert_W02[:,1],linewidth = .5)
plt.plot(vert_W03[:,0],vert_W03[:,1],linewidth = .5)
plt.plot(vert_W04[:,0],vert_W04[:,1],linewidth = .5)
plt.plot(vert_W05[:,0],vert_W05[:,1],linewidth = .5)
plt.plot(vert_W06[:,0],vert_W06[:,1],linewidth = .5)
plt.plot(vert_W07[:,0],vert_W07[:,1],linewidth = .5)
plt.plot(vert_W08[:,0],vert_W08[:,1],linewidth = .5)
plt.plot(vert_W09[:,0],vert_W09[:,1],linewidth = .5)
plt.scatter(q_start[0],q_start[1])
plt.scatter(q_goal[0],q_goal[1])
plt.plot(path[:,0],path[:,1],color = 'black',linewidth = .5)
#plt.scatter(xv[x_store != 1],yv[y_store != 1],s=.25)

diff = np.array([])
t  = len(path)
for i in range(len(path)-1):
    d = path[i+1] - path[i]
    total = np.linalg.norm(d)
    diff = np.append(diff,total)
    


Total_Traveled = np.sum(diff)
print("Total Length of Path is", Total_Traveled)