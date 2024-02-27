# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 14:07:32 2022

@author: riana
"""

#how in the damn hell

#plot the C space part a
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d

def linprim(q,O): #need this (no change)
    
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

#Base of the robot will be at 0,0
#robot will be a two link manipulator 
#link 2 will connect to link 1
#theta 2 is theta with respect to link 1
#theta 1 is theta with respect to the origin

#C space will have theta 1 and theta 2 on its axis

#User can specify:

# length of each link   
l1 = 1
l2 = 1

#number of Obstacles
numOb = 1


#vertices of each obstacle
a1 = [.25,.25] 
a2 = [0,.75]
a3 = [-.25,.25]

O = np.array([a1,a2,a3,a1]) # obstacle verts

#iterate through theta1 and theta 2 0 to 2pi for each link
t1 = np.linspace(0, 360, 1441) #using radians
t2 = np.linspace(0, 360, 1441)

C_space = np.array([])

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
            
    

t = len(C_space)/2

C_space = C_space.reshape(int(t),2)

hull = ConvexHull(C_space)



#plt.plot(C_space[hull.vertices,0], C_space[hull.vertices,1], 'r--', lw=2)
#plt.plot(C_space[hull.vertices[0],0], C_space[hull.vertices[0],1], 'ro')
plt.plot(C_space[:,0], C_space[:,1], 'o')
plt.title('C Space 3a')
plt.show()
#use convex hull to create the points on a graph to create the c space 

    
    
    
    
    
    
