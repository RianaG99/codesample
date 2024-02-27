# -*- coding: utf-8 -*-
"""
Created on Mon Oct  3 16:13:47 2022

@author: riana

"""

#initialize the obstacles in the workspace as 1

#take in the collision points and correlate to , not being used here
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
                       
                    if xv[i][j] + step in xv or xv[i][j] == 2*np.pi:
                        if xv[i][j] != 2*np.pi:
                            p = np.argwhere(xv == xv[i][j] + step)
                            q = np.argwhere(yv == yv[i][j])
                            ly = p[2,1]
                            lx = q[2,0]
                        elif xv[i][j] == 2*np.pi:
                            p = np.argwhere(xv == 0)
                            q = np.argwhere(yv == yv[i][j])
                            ly = p[2,1]
                            lx = q[2,0]
                
                             
                        if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                             x_store[lx,ly] = counter + 1
                             y_store[lx,ly] = counter + 1
                        else:
                             x_store[lx,ly] = x_store[lx,ly]
                             y_store[lx,ly]= y_store[lx,ly]
                       
                         
                    if xv[i][j] - step in xv or xv[i][j] == 0:
                        if xv[i][j] != 0:
                            p = np.argwhere(xv == xv[i][j] - step)
                            q = np.argwhere(yv == yv[i][j])
                            ly = p[2,1]
                            lx = q[2,0]
                        elif xv[i][j] == 0:
                            p = np.argwhere(xv == 2*np.pi)
                            q = np.argwhere(yv == yv[i][j])
                            ly = p[2,1]
                            lx = q[2,0]
                        
                             
                        if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                            x_store[lx,ly] = counter + 1
                            y_store[lx,ly] = counter + 1
                        else:
                            x_store[lx,ly] = x_store[lx,ly]
                            y_store[lx,ly]= y_store[lx,ly]
                                 
                    if yv[i][j] + step in yv or yv[i][j] == 2*np.pi:
                        if yv[i][j] != 2*np.pi:
                            p = np.argwhere(xv == xv[i][j])
                            q = np.argwhere(yv == yv[i][j] + step)
                            ly = p[2,1]
                            lx = q[2,0]
                        elif yv[i][j] == 2*np.pi:
                            p = np.argwhere(xv == xv[i][j])
                            q = np.argwhere(yv == 0)
                            ly = p[2,1]
                            lx = q[2,0]
                             
                        if x_store[lx,ly] == 0 and y_store[lx,ly] == 0:
                             x_store[lx,ly] = counter + 1
                             y_store[lx,ly] = counter + 1
                        else:
                             x_store[lx,ly] = x_store[lx,ly]
                             y_store[lx,ly]= y_store[lx,ly]
                         
                    if yv[i][j] - step in yv or yv[i][j] == 0:
                        if yv[i][j] != 0:
                            p = np.argwhere(xv == xv[i][j])
                            q = np.argwhere(yv == yv[i][j] - step)
                            ly = p[2,1]
                            lx = q[2,0]
                        elif yv[i][j] == 0:
                            p = np.argwhere(xv == xv[i][j])
                            q = np.argwhere(yv == 2*np.pi)
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
def linprim(q,O):  #define the C_space from the WS, no need to change
    
    #input : Obstacle [x,y] (in counter clockwise order, include first vert twice)
    #input : q [x,y] coordinate along the line
    
    numVert = len(O)
    x = q[0] #x position of the path
    y = q[1] #y position of the path

    if O.ndim == 2:
        
       a = np.array([])
       for i in range(numVert-1):
           
           H = -((O[i,1]- O[i+1,1])*x+(O[i+1,0]- O[i,0])*y + (O[i,0]*O[i+1,1]- O[i+1,0]*O[i,1]))
        
           if H < 0:
               a = np.append(a,1)  
               
           if H > 0 :
               a = np.append(a,0)
               collide = 0
               break
           
       if a.all() == True:
           collide = 1

           
    if O.ndim == 3:
        
        numOb= len(O)
        for i in range(numOb):
            a = np.array([])
            
            numVert = len(O[i])
            for j in range(numVert-1):
                H = -((O[i][j,1]- O[i][j+1,1])*x+(O[i][j+1,0]- O[i][j,0])*y + (O[i][j,0]*O[i][j+1,1]- O[i][j+1,0]*O[i][j,1]))
                
                if H < 0:
                    a = np.append(a,1)
                if H > 0:
                    a = np.append(a,0)
                    collide = 0
                    break
            
            if a.all() == True :
                collide = 1
                
          
    return collide
                
           
#search for the optimal path


import numpy as np
import matplotlib.pyplot as plt
import math
import random
import matplotlib.animation as ani


step = np.pi/10


#created a mesh grid of points between 0 and .25
#nx, ny = (1441,1441) #dont need a wave front in the Workspace
t1 = np.arange(0,2*np.pi+step, step)#np.linspace(0, 360, nx)*np.pi/180
t2 = np.arange(0, 2*np.pi+step, step)#np.linspace(0, 360, ny)*np.pi/180
t1v, t2v = np.meshgrid(t1, t2)

#store the values of the wavefront in here that correspond to the point on meshgrid
x_store = t1v*0
y_store= t2v*0

#initial values

l1 = 1 #length of link 1
l2 = 1 #length of link 2
q_base = [0,0]

q_link2 = [-1,0] #beginning position of the second link
q_start = [-2,0] #beginning position of end effectore with base at 0,0

theta_start = [np.pi,0] #starting position t1,t2 starting positon
link_start = np.array([q_base,q_link2,q_start])
q_goal = [2,0] # end potision of end effector base still 0,0
theta_end = [0,0]

#initialize the goal position points as 2 in our 
#for i in range(len(xv)):
 #   for j in range(len(yv)):
  #      if xv[i][j] == q_goal[0] and yv[i][j] == q_goal[1]:
   #         x_store[i][j] = 2
    #        y_store[i][j] = 2
            
            
numOb = 2


#vertices of each obstacle
a1 = [-.25,1.1] 
a2 = [.25,1.1]
a3 = [.25,2]
a4 = [-.25,2]
O1 = np.array([a1,a2,a3,a4,a1]) 

b1 = [-2,-.5] 
b2 = [2,-.5]
b3 = [2,-.3]
b4 = [-2,-.3]

O2 = np.array([b1,b2,b3,b4,b1]) 

O = np.array([O1,O2])

#x_store and y_store now know which points correspond to points in an obstacle
#x_store,y_store = intialize(o1, xv, yv, x_store, y_store)

#the completed wavefront, x_store and y_store should be identical
#x_store,y_store = TheWave(xv,yv,x_store,y_store,step)

C_space = np.array([])

#this gives me my C_space points [t1,t2]
for s in range(numOb):

    for i in range(len(t1)):
    
        for j in range(len(t2)):
#Finding the end of effectors (this work!)
            p = np.array([[0], [0], [1]]) #what to multiple by to find rotated link
            T1 = np.array([[np.cos(t1[i]), -np.sin(t1[i]), 0],[np.sin(t1[i]), np.cos(t1[i]), 0],[ 0, 0, 1]])
            end_ef1 = np.dot(T1,p) #end effector of joint 1 , just 0,0 at base
            T2 = np.array([[np.cos(t2[j]), -np.sin(t2[j]), l1],[np.sin(t2[j]), np.cos(t2[j]), 0],[ 0, 0, 1]])
            end_ef2 = np.linalg.multi_dot([T1,T2,p])
            T3 = np.array([[1, 0, l2], [0, 1, 0], [0, 0, 1]])
            end_ef3 = np.linalg.multi_dot([T1,T2,T3,p]) #end effector of joint 3
        #now all end effectors are known

        #Next, use the vert points of each to find points along the line(Works!)
            numpoints = 10; #number of points along the line

        #store points for bars 1 and 2 in these arrays
            ql1 = np.zeros(shape =(10,2)) #points along link 1
            ql2 = np.zeros(shape =(10,2)) #points along link 2

            for k in range(numpoints):
    #loop through to find 10 points along link 1
                ql1[k,0] = (k/numpoints)*np.cos(t1[i])
                ql1[k,1] = (k/numpoints)*np.sin(t1[i])
            
            
    

            for n in range(numpoints):
     #loop through to find all points along link 2 
                ql2[n,0] = end_ef2[0] + n/numpoints*np.cos(t1[i]+t2[j])
                ql2[n,1] = end_ef2[1] + n/numpoints*np.sin(t1[i]+t2[j])
            
        #go into collision function that should mostly work right now
        
            collision1 = np.zeros(10) #initialize arrays for the collisional 
            collision2 = np.zeros(10)
        
            if numOb == 1 :
        
                for col in range(numpoints):   
                    collision1[col] = linprim(ql1[col],O)
                    collision2[col] = linprim(ql2[col],O)
            else:
                for col in range(numpoints):   
                    collision1[col] = linprim(ql1[col],O[s])
                    collision2[col] = linprim(ql2[col],O[s])
            
        #store the value of theta 1 and theta 2 if there is a collision
            if collision1.any() == True or collision2.any() == True:
                C_space = np.append(C_space,[t1[i],t2[j]])
            
#C_space collision points correlate to 1            
    

t = len(C_space)/2

C_space = C_space.reshape(int(t),2)

for i in range(len(t1v)):
    for j in range(len(t2v)):
        if t1v[i][j] == theta_end[0] and t2v[i][j] == theta_end[1]:
            x_store[i][j] = 2
            y_store[i][j] = 2

#define the points of collision as 1
#this allows my grid to be up to date with t1,t2 points now
for k in range(len(C_space)):
    for i in range(len(t1v)):
        for j in range(len(t2v)):
            if t1v[i][j] == C_space[k][0] and t2v[i][j] == C_space[k][1]:
                x_store[i][j] = 1
                y_store[i][j] = 1
                
            
            
#must be able to correlate a value between 0 and 2*pi- i e make them equal/next to each other
x_store,y_store = TheWave(t1v,t2v,x_store,y_store,step) #make the wave

path = np.array([])

p_start = np.argwhere(t1v == np.pi)
q_start = np.argwhere(t2v == 0)
ly_start= p_start[2,1] #rnadom chosen values, the y corresponds to any y in this array
lx_start = q_start[2,0]#rnadom chosen values, the y corresponds to any x in this array

start_value = x_store[lx_start,ly_start]
q_start = [t1v[lx_start,ly_start],t2v[lx_start,ly_start]]
path = np.append(path,q_start)
i = 1
q = q_start
grid = start_value

#define the path
while (grid - (i-1)) != 2:
    
    #have the value of goal 
    step_x = t1v[x_store == grid - i]
    step_y = t2v[x_store == grid - i]
    
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

#store the values of theta 1 and theta 2 as the path taken by the arm

#simulate the arm moving at each of these theta points

i = 24


p = np.array([[0], [0], [1]]) #what to multiple by to find rotated link
T1 = np.array([[np.cos(path[i][0]), -np.sin(path[i][0]), 0],[np.sin(path[i][0]), np.cos(path[i][0]), 0],[ 0, 0, 1]])
end_ef1 = np.dot(T1,p) #end effector of joint 1 , just 0,0 at base
T2 = np.array([[np.cos(path[i][1]), -np.sin(path[i][1]), l1],[np.sin(path[i][1]), np.cos(path[i][1]), 0],[ 0, 0, 1]])
end_ef2 = np.linalg.multi_dot([T1,T2,p])
T3 = np.array([[1, 0, l2], [0, 1, 0], [0, 0, 1]])
end_ef3 = np.linalg.multi_dot([T1,T2,T3,p]) #end effector of joint 3
   
    #plt.plot(O[:,0],O[:,1])
    
move = np.array([end_ef1,end_ef2,end_ef3])


plt.scatter(move[:,0],move[:,1])
plt.plot(move[:,0],move[:,1])
plt.scatter(q_goal[0],q_goal[1])
plt.plot(O2[:,0],O2[:,1])
plt.plot(O1[:,0],O1[:,1])





#plt.plot(C_space[:,0], C_space[:,1], 'o')
#plt.scatter(t1v,t2v,color = 'black', s = .25)
#plt.plot(path[:,0],path[:,1],color = 'black',linewidth = .75)
#plt.scatter(t1v[x_store !=1],t2v[y_store!=1],color = 'black') 
#plt.scatter(theta_start[0],theta_start[1], color ='orange')
#plt.scatter(theta_end[0],theta_end[1],color = 'red')
#plt.title('C Space 3b')
#plt.plot(C_space[x_store != 1], C_space[:,1], 'o')
#plt.show()
