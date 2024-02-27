# -*- coding: utf-8 -*-
"""
Created on Wed Oct 12 14:35:54 2022

@author: riana
"""

def generateNode(x_min,x_max,y_min,y_max,Q,q_goal):
    
    chance = np.random.uniform(0,1,1)
    
    if chance < Q:
        q_randx = random.uniform(x_min,x_max)
        q_randy = random.uniform(y_min,y_max)
        q_rand = np.array([q_randx,q_randy])
        
    elif chance >= Q:
        q_rand = q_goal
        
    return q_rand

def generatePath(q_rand,V,r):
    
    point_alongx = np.array([])
    point_alongy = np.array([])
    
    dists = np.array([])
    if len(V) > 1:
        for i in range(len(V)):
            dists = np.append(dists,math.dist(q_rand,V[i]))

            
            
        q_near = V[np.where(dists == np.min(dists))[0][0]] #closest node to q_rand
            
        along_x = np.linspace(q_near[0],q_rand[0],20)
        along_y = np.linspace(q_near[1],q_rand[1],20)
        for i in range(len(along_x)):
            leng = math.dist([along_x[i],along_y[i]],q_near)
            if leng <= .5:
                point_alongx = np.append(point_alongx,along_x[i])
                point_alongy = np.append(point_alongy,along_y[i])
        


            
        q_new =  np.array([point_alongx[-1],point_alongy[-1]]) #point r away from q_near along path
            
    if len(V) == 1:
        q_near = V[0]
        along_x = np.linspace(q_near[0],q_rand[0],20)
        along_y = np.linspace(q_near[1],q_rand[1],20)
        for i in range(len(along_x)):
            leng = math.dist([along_x[i],along_y[i]],q_near)
            if leng <= .5:
                point_alongx = np.append(point_alongx,along_x[i])
                point_alongy = np.append(point_alongy,along_y[i])
        

            
        q_new = np.array([point_alongx[-1],point_alongy[-1]]) #point r away from q_near along path
        
            
            
    return q_near,q_new

def CollisionFree(q_near,q_new,O):
    
    sample = 5 #take 5 points along the path
    

    along_x = np.linspace(q_near[0],q_new[0],sample)
    along_y = np.linspace(q_near[1],q_new[1],sample)
    

    if O.ndim == 2:
        numVert = len(O)
               
        for k in range(sample):
           
            a = np.array([])
            for q in range(numVert-1):
                    
                H = -((O[q,1]- O[q+1,1])*along_x[k]+(O[q+1,0]- O[q,0])*along_y[k] + (O[q,0]*O[q+1,1]- O[q+1,0]*O[q,1])) -.5
                
                if H < 0:
                    a = np.append(a,1)  
                       
                if H > 0 :
                    a = np.append(a,0)
                    break
                       
            if a.all():
                Collision = 1
                break
            else:
                Collision = 0

             #this is for the 3D obstacles      
    if O.ndim == 3:
                
        numOb= len(O)
        for q in range(numOb):
            numVert = len(O[q])
            for k in range(len(along_x)):
                a = np.array([])           
                for j in range(numVert-1):
                  
                    H = -((O[q][j,1]- O[q][j+1,1])*along_x[k]+(O[q][j+1,0]- O[q][j,0])*along_y[k] + (O[q][j,0]*O[q][j+1,1]- O[q][j+1,0]*O[q][j,1])) 
               
                    if H < 0:
                        a = np.append(a,1)  
                  
                    if H > 0 :
                        a = np.append(a,0)
                        break
                        
                if a.all():
                    Collision = 1
                    break
                else:
                    Collision = 0
    
    edge = math.dist(q_near,q_new)
    
    return Collision,edge
    
    
    
    
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import time

start_time = time.time()

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

p = .05
Q = 1-p #chance it takes q_goal as the node
q_start = [0,0]
q_goal = [10,0]
e = .25
r = .5 #step away from q_near
n = 5000

y_min = -3
y_max = 3
x_min = -1
x_max = 11

T = np.array([]) #Tree containing pairing nodes and edge
V = np.array([q_start]) #existing vertices
final_dist = math.dist(q_start,q_goal)
nodes = 1

while (final_dist > e) and (nodes < n):
    
    q_rand = generateNode(x_min, x_max, y_min, y_max, Q, q_goal)
    q_near,q_new = generatePath(q_rand,V,r)
    collision1, edge = CollisionFree(q_near,q_new,o1)
    collision2, edge = CollisionFree(q_near,q_new,o2)
    
    if collision1 == 0 and collision2 == 0:
        V = np.append(V,[q_new])
        tv = int(len(V)/2)
        V = V.reshape(tv,2)
        T = np.append(T,[q_near,q_new,edge])
        tt = int(len(T)/3)
        T = T.reshape(tt,3)
        final_dist = math.dist(q_new,q_goal)
        nodes = len(V)



path = V[-1]
node = V[-1]
cost = np.array([])

while any(node != q_start):
    for i in range(len(T)):
        if all(T[i][1] == node):
            node = T[i][0]
            path = np.append(path,node)
            l = T[i][2]
            cost = np.append(cost,l)
            
            
tp = int(len(path)/2)
path = path.reshape(tp,2)   
#path = np.flip(path)       
path_length = np.sum(cost)  
end_time = time.time()

total_time = end_time-start_time #seconds   


plt.plot(o1[:,0],o1[:,1])
plt.plot(o2[:,0],o2[:,1])
plt.scatter(V[:,0],V[:,1])
plt.scatter(q_goal[0],q_goal[1])
plt.scatter(q_start[0],q_start[1])
plt.plot(path[:,0],path[:,1])
plt.title(f"Path Length is {path_length}")

    
