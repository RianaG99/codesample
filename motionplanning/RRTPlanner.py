# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 23:04:50 2022

@author: riana
"""
#create samples in the free space that don't collide
#create randomized x and y points between a min and max
def Sample(x_min,x_max,y_min,y_max,n): #works
    #create random samples all over C_space
    x = np.random.uniform(x_min,x_max+1,n)
    y = np.random.uniform(y_min,y_max+1,n)
    
    return x,y

#check if the x and y points collide and take them out, creating new x and y points
def IsSampleCollisionFree(x,y,O): #works
    
    x_safe = x
    y_safe = y
    
    #this is for the 2D obstacles
    if O.ndim == 2:
       numVert = len(O)
       
       for k in range(len(x)):
           a = np.array([])
           for i in range(numVert-1):
               
           
               H = -((O[i,1]- O[i+1,1])*x[k]+(O[i+1,0]- O[i,0])*y[k] + (O[i,0]*O[i+1,1]- O[i+1,0]*O[i,1]))
        
               if H < 0:
                   a = np.append(a,1)  
               
               if H > 0 :
                   a = np.append(a,0)
                   break
               
           if a.all():
               x[k] = 20
               y[k] = 20
                    
  
     #this is for the 3D obstacles      
    if O.ndim == 3:
        
        numOb= len(O)
        for i in range(numOb):
            numVert = len(O[i])
            for k in range(len(x)):
                a = np.array([])           
                for j in range(numVert-1):
          
                    H = -((O[i][j,1]- O[i][j+1,1])*x[k]+(O[i][j+1,0]- O[i][j,0])*y[k] + (O[i][j,0]*O[i][j+1,1]- O[i][j+1,0]*O[i][j,1]))
       
                    if H < 0:
                        a = np.append(a,1)  
              
                    if H > 0 :
                        a = np.append(a,0)
                        break
                if a.all():
                    x[k] = 20 #20 is a placeholder value outside the range of the WS
                    y[k] = 20
                    #if all points are 1 there is a collision
    x_safe = np.delete(x,np.where(x == 20)[0])
    y_safe = np.delete(y,np.where(y == 20)[0])
      #take out the bad poibts          

    return x_safe,y_safe

import numpy as np
import matplotlib.pyplot as plt
#import random
import math
import time
#inputs:
    
start_time = time.time()
    
n = 5000
r = 1
pgoal = .05  #qrand is qgoal 5% of the time
e = .25
q_start = [0,0]
q_goal = [10,0]
Q = 1-pgoal

y_min = -3
y_max = 3
x_min = -1
x_max = 11

v1 = [3.5,0.5]
v2 = [4.5,0.5]
v3 = [4.5,1.5]
v4 = [3.5,1.5]
o1 = np.array([v1,v2,v3,v4,v1])

#centered at (7,-1)
w1 = [6.5,-1.5]
w2 = [7.5,-1.5]
w3 = [7.5,-0.5]
w4 = [6.5,-0.5]
o2 = np.array([w1,w2,w3,w4,w1])

x,y = Sample(x_min,x_max,y_min,y_max,n)

#append start and goal to your list of points as the first and last point
x= np.insert(x,0,q_start[0])
y= np.insert(y,0,q_start[1])
x= np.append(x,q_goal[0])
y= np.append(y,q_goal[1])

x,y = IsSampleCollisionFree(x,y,o1)
x_safe,y_safe = IsSampleCollisionFree(x,y,o2)

Tree = np.array([q_start]) #initialize tree at q init

