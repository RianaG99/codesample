# -*- coding: utf-8 -*-
"""
Created on Tue Oct 18 15:48:13 2022

@author: riana
"""

def generateNode(x_min,x_max,y_min,y_max,Q,q_goal): #this is fine, only work with 1 robot


    
    chance = np.random.uniform(0,1,1)
 
    if chance < Q:
        q_randx = random.uniform(x_min,x_max)
        q_randy = random.uniform(y_min,y_max)
        q_rand = np.array([q_randx,q_randy])
        
    elif chance >= Q:
        q_rand = q_goal
        
    return q_rand



def generatePath(q_rand,V,r,time):
    #inputs: V: all nodes, r: distance from node, q_rand: random value 
    #output: 
       
    dists = np.array([])
    
    if len(V) > 1:
        for i in range(len(V)):
            dists = np.append(dists,np.linalg.norm(q_rand-V[i]))
            
        q_near = V[np.where(dists == np.min(dists))[0][0]] #closest node to q_rand
        q_near = V[np.where(dists == np.min(dists))[0][0]]
        time_near = time[np.where(dists == np.min(dists))[0][0]] #time of parent node
        new_time = time_near + 1 #time of neighboring node
        l = ((q_rand-q_near)/np.linalg.norm(q_rand-q_near))  
        q_new =  q_near + l*r #point r away from q_near along path
            
    if len(V) == 1:
        q_near = V[0]
        time_near = time[0]
        new_time = time_near+1 #add one second between nodes
        l = ((q_rand-q_near)/np.linalg.norm(q_rand-q_near))    
        q_new = q_near + l*r #point r away from q_near along path
        

    return q_near,q_new,new_time,time_near #assign the time here
    

def CollisionFree(q_near,q_new,O,R): #independent of time
    
    sample = 10 #take 5 points along the path
    

    along_x = np.linspace(q_near[0],q_new[0],sample)
    along_y = np.linspace(q_near[1],q_new[1],sample)
                
    numOb= len(O)
    for q in range(numOb):
        numVert = len(O[q])
        for k in range(sample):
            a = np.array([])           
            for j in range(numVert-1):
                  
                H = -((O[q][j,1]- O[q][j+1,1])*along_x[k]+(O[q][j+1,0]- O[q][j,0])*along_y[k] + (O[q][j,0]*O[q][j+1,1]- O[q][j+1,0]*O[q][j,1])) - 2*R
           
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



def pathCreations(V,q_start,T):  
    
    path = V[-1]
    node = V[-1]
    time_travel = np.array([T[-1][4]])
    while any(node != q_start):
        for i in range(len(T)):
            if all(T[i][1] == node):
                node = T[i][0]
                path = np.append(path,node)
                t = T[i][3] #put final time at end of tree
                time_travel = np.append(time_travel,t)
                
                
    tp = int(len(path)/2)
    path = path.reshape(tp,2) 
    time_travel = time_travel.reshape(int(len(time_travel)),1)
    
    
    
    return path,time_travel #return time also? PROBABLY
    

def robotCollision(q_near, q_new,R,new_time,time_near,path,times,m):
    #path is all the agent times
    #times is all the agent times
    
    #generate points along q_near and q_new
    sample = 5 #take 5 points along the path
    along_x = np.linspace(q_near[0],q_new[0],sample)
    along_y = np.linspace(q_near[1],q_new[1],sample)
    
    for i in range(m):
    
        
        if times[m][0] < new_time:
            Collision = 0
            
        else:
            ind_near = np.where(times[i] == time_near)[0][0]
            ind_new = np.where(times[i] == new_time)[0][0]
            agent_near = path[i][ind_near]
            agent_new = path[i][ind_new]
        
            along_xa = np.linspace(agent_near[0],agent_new[0],sample)
            along_ya = np.linspace(agent_near[1],agent_new[1],sample)
        
            for k in range(sample):
                robot_distance = math.dist([along_x[k],along_y[k]],[along_xa[k],along_ya[k]])
            
                if robot_distance <= 2*R:
                    Collision = 1
                    break
                else:    
                    Collision = 0
        
            if Collision == 1:
                break
    
    
    
    
    return Collision
    
    
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import time

v1 = [4,6]
v2 = [6,6]
v3 = [6,7]
v4 = [4,7]
o1 = np.array([v1,v2,v3,v4,v1])

#centered at (7,-1)
w1 = [4,6]
w2 = [5,6]
w3 = [5,10]
w4 = [4,10]
o2 = np.array([w1,w2,w3,w4,w1])

z1 = [4,9]
z2 = [6,9]
z3 = [6,10]
z4 = [4,10]
o3 = np.array([z1,z2,z3,z4,z1])

#centered at (7,-1)
a1 = [10,6]
a2 = [12,6]
a3 = [12,7]
a4 = [10,7]
o4 = np.array([a1,a2,a3,a4,a1])

b1 = [11,6]
b2 = [12,6]
b3 = [12,10]
b4 = [11,10]
o5 = np.array([b1,b2,b3,b4,b1])

#centered at (7,-1)
c1 = [10,9]
c2 = [12,9]
c3 = [12,10]
c4 = [10,10]
o6 = np.array([c1,c2,c3,c4,c1])

o = np.array([o1,o2,o3,o4,o5,o6])



start_time = time.time()
p = .05
Q = 1-p #chance it takes q_goal as the node
e = .25
r = .5 #step away from q_near
n = 7500

#WILL HAVE m number of robots
#will have to test points around the robot instead of just point
R = .5 #every robot is a disk with radius

#goal and start given by center of robot

x1_start = [2,2]
x1_goal = [14,14]

x2_start = [2,14]
x2_goal = [14,2]

x3_start = [8,14]
x3_goal = [8,2]

x4_start = [2,8]
x4_goal = [14,8]

x5_start = [11,2]
x5_goal = [5,14]

x6_start = [11,14]
x6_goal = [5,2]




starts = np.vstack((x1_start,x2_start,x3_start,x4_start,x5_start,x6_start))
goals = np.vstack((x1_goal,x2_goal,x3_goal,x4_goal,x5_goal,x6_goal))

#
y_min = 0
y_max = 16
x_min = 0
x_max = 16

#tag the time function to the end of a tree, one second per travel edge
agents = 2
times = np.array([])
paths = np.array([])
m1 = np.zeros([7500,2])
m2 = np.zeros([7500,2])
m3 = np.zeros([7500,2])
m4 = np.zeros([7500,2])
m5 = np.zeros([7500,2])
m6 = np.zeros([7500,2])

t1 = np.zeros([7500,1])
t2 = np.zeros([7500,1])
t3 = np.zeros([7500,1])
t4 = np.zeros([7500,1])
t5 = np.zeros([7500,1])
t6 = np.zeros([7500,1])

#change this for agents?
agent_paths = np.array([m1,m2,m3,m4,m5,m6])
agent_times = np.array([t1,t2,t3,t4,t5,t6])


for m in range(agents):
    q_start = starts[m] 
    q_goal = goals[m]
    T = np.array([]) #Tree containing pairing nodes and edge
    V = np.array([q_start]) #existing
    time_1robot = np.array([0]) #concurrently make a time func, index of time will be time node occurs
    final_dist = math.dist(q_start,q_goal)
    nodes = 1
    while (final_dist > e) and (nodes < n):
    
        q_rand = generateNode(x_min, x_max, y_min, y_max, Q, q_goal)
        q_near,q_new, new_time ,time_near = generatePath(q_rand,V,r,time_1robot)
        collisionob, edge = CollisionFree(q_near,q_new,o,R)
        
        if m >= 1:
            collisionRob = robotCollision(q_near, q_new,R,new_time,time_near,agent_paths,agent_times,m)
        else:
            collisionRob = 0
        
    
        if collisionob == 0: # and collisionRob == 0:
            V = np.append(V,[q_new]) #append the time of just that node
            time_1robot = np.append(time_1robot,new_time)
            tv = int(len(V)/2)
            V = V.reshape(tv,2)
            T = np.append(T,[q_near,q_new,edge,time_near,new_time])#append the time of parent and neighbor
            tt = int(len(T)/5)
            T = T.reshape(tt,5)
            final_dist = math.dist(q_new,q_goal)
            nodes = len(V)
            #dont need to save V, just the path and time will be important
            
    path,time_travel = pathCreations(V, q_start,T) #STORE THIS FOR EACH
    
    agent_paths[m][0:int(len(path))] = path
    agent_paths[m][int(len(path)):-1] = None
    agent_paths[m][-1] = None
    agent_times[m][0:int(len(time_travel))] = time_travel
    agent_times[m][int(len(time_travel)):-1] = None
    agent_times[m][-1] = None
        

    
end_time = time.time()
total_time = end_time-start_time #seconds  

agent_1path = agent_paths[0][~np.isnan(agent_paths[0])]  
shape1 =int(len(agent_1path)/2)
agent_1path=agent_1path.reshape(shape1,2)

agent_2path = agent_paths[1][~np.isnan(agent_paths[1])]  
shape2 =int(len(agent_2path)/2)
agent_2path = agent_2path.reshape(shape2,2)

agent_3path = agent_paths[2][~np.isnan(agent_paths[2])]  
shape3 =int(len(agent_3path)/2)
agent_3path = agent_3path.reshape(shape3,2)

agent_4path = agent_paths[3][~np.isnan(agent_paths[3])]  
shape4 =int(len(agent_4path)/2)
agent_4path=agent_4path.reshape(shape4,2)

agent_5path = agent_paths[4][~np.isnan(agent_paths[4])]  
shape5 =int(len(agent_5path)/2)
agent_5path = agent_5path.reshape(shape5,2)

agent_6path = agent_paths[5][~np.isnan(agent_paths[5])]  
shape6 =int(len(agent_6path)/2)
agent_6path = agent_6path.reshape(shape6,2)

plt.plot(o1[:,0],o1[:,1],linewidth=.75)
plt.plot(o2[:,0],o2[:,1],linewidth=.75)
plt.plot(o3[:,0],o3[:,1],linewidth=.75)
plt.plot(o4[:,0],o4[:,1],linewidth=.75)
plt.plot(o5[:,0],o5[:,1],linewidth=.75)
plt.plot(o6[:,0],o6[:,1],linewidth=.75)

plt.scatter(x1_goal[0],x1_goal[1])
plt.scatter(x1_start[0],x1_start[1])
plt.scatter(x2_goal[0],x2_goal[1])
plt.scatter(x2_start[0],x2_start[1])

plt.plot(agent_1path[:,0],agent_1path[:,1],linewidth=.75,color = 'red')
plt.plot(agent_2path[:,0],agent_2path[:,1],linewidth=.75,color = 'red')


