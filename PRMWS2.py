# -*- coding: utf-8 -*-
"""
Created on Sun Oct  9 18:57:57 2022

@author: riana
"""
#This is the PRM Planner for WS 2 in HW 2 exercise 2

#sample n configurations

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

def GenerateLocalPath(x_safe,y_safe,r): #works
    
    #store as [ind1,ind2,dist]
    #ind 1 is the parent ind 2 is the neighbor dist is the distance between them
    neighbors = np.array([])

    for i in range(len(x_safe)):
        for j in range(len(y_safe)):
            
            dist = math.dist([x_safe[i],y_safe[i]],[x_safe[j],y_safe[j]])
            
            if dist <= r and i !=j: #and x_safe[j]>=x_safe[i] and y_safe[j]>=y_safe[i]:
                
                neighbors = np.append(neighbors,[i,j,dist])

    t = int(len(neighbors)/3)        
    neighbors = neighbors.reshape(t,3)   
    

    return neighbors

def IsPathCollisionFree(neighbors,x_safe,y_safe,O): #works
    
    paths_new = neighbors
    bad_inds = np.zeros(len(neighbors))
    
    for i in range(len(neighbors)):
        along_x = np.linspace(x_safe[int(neighbors[i][0])],x_safe[int(neighbors[i][1])],30)
        along_y = np.linspace(y_safe[int(neighbors[i][0])],y_safe[int(neighbors[i][1])],30)
        
        
        if O.ndim == 2:
            numVert = len(O)
               
            for k in range(len(along_x)):
           
                a = np.array([])
                for q in range(numVert-1):
                    
                    H = -((O[q,1]- O[q+1,1])*along_x[k]+(O[q+1,0]- O[q,0])*along_y[k] + (O[q,0]*O[q+1,1]- O[q+1,0]*O[q,1]) +.25)
                
                    if H < 0:
                        a = np.append(a,1)  
                       
                    if H > 0 :
                        a = np.append(a,0)
                        break
                       
                if a.all():
                     
                     bad_inds[i] = 1
                     break

             #this is for the 3D obstacles      
        if O.ndim == 3:
                
                numOb= len(O)
                for q in range(numOb):
                    numVert = len(O[q])
                    for k in range(len(along_x)):
                        a = np.array([])           
                        for j in range(numVert-1):
                  
                            H = -((O[q][j,1]- O[q][j+1,1])*along_x[k]+(O[q][j+1,0]- O[q][j,0])*along_y[k] + (O[q][j,0]*O[q][j+1,1]- O[q][j+1,0]*O[q][j,1])+.25)
               
                            if H < 0:
                                a = np.append(a,1)  
                      
                            if H > 0 :
                                a = np.append(a,0)
                                break
                        if a.all():
                            bad_inds[i] = 1
                            
                            break
               

                
                            
                            
    
    
    paths_new = np.delete(neighbors,np.where(bad_inds == 1)[0],axis = 0)

    
                    
    #create points along the line
    #check if any of the points are in collision
    #if true, throw out
    #retun new paths that work
    
    return paths_new

def CollisionFree(q_near,q_new,O):
    
    
    
    sample = 30 #take 5 points along the path
    

    along_x = np.linspace(q_near[0],q_new[0],sample)
    along_y = np.linspace(q_near[1],q_new[1],sample)
    

    if O.ndim == 2:
        numVert = len(O)
               
        for k in range(sample):
           
            a = np.array([])
            for q in range(numVert-1):
                    
                H = -((O[q,1]- O[q+1,1])*along_x[k]+(O[q+1,0]- O[q,0])*along_y[k] + (O[q,0]*O[q+1,1]- O[q+1,0]*O[q,1]) +.25)
                
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
            for k in range(sample):
                a = np.array([])           
                for j in range(numVert-1):
                  
                    H = -((O[q][j,1]- O[q][j+1,1])*along_x[k]+(O[q][j+1,0]- O[q][j,0])*along_y[k] + (O[q][j,0]*O[q][j+1,1]- O[q][j+1,0]*O[q][j,1]) +.25)
               
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
            if a.all():
                break
                    
    edge = math.dist(q_near,q_new)
    
    return Collision,edge


def pathsmooth(x_path,y_path,o1):
    
    
    smooth_pathx = np.array([x_path[0]])
    smooth_pathy = np.array([y_path[0]])
    trackingx = np.array([]) #put in loop so it res
    trackingy = np.array([])
    l = np.array([])
    edge_tracking = np.array([])
    final_nodex = 0
    final_nodey = 0
    
    i = 0
    while final_nodex != x_path[-1] or final_nodey != y_path[-1]:
        current = np.array([x_path[i],y_path[i]])
        
        collision1,edge = CollisionFree([smooth_pathx[-1],smooth_pathy[-1]],current,o1)

        if (collision1 == 0) and (current[0] != x_path[-1] or current[1] != y_path[-1] ):
            trackingx = np.append(trackingx,current[0])
            trackingy = np.append(trackingx,current[1])
            edge_tracking = np.append(edge_tracking,edge)
            i = i+1
        elif (collision1 == 1):
            smooth_pathx = np.append(smooth_pathx,trackingx[-1])
            smooth_pathy = np.append(smooth_pathy,trackingy[-1])
            l = np.append(l,edge_tracking[-1])
            final_nodex = smooth_pathx[-1]
            final_nodey = smooth_pathy[-1]
            i = i
        elif (current[0] == x_path[-1] and current[1] == y_path[-1] ):
            smooth_pathx = np.append(smooth_pathx, current[0])
            smooth_pathy = np.append(smooth_pathy,current[1])
            l = np.append(l,edge)
            final_nodex = smooth_pathx[-1]
            final_nodey = smooth_pathy[-1]
        
    l = np.sum(l)
    
    return smooth_pathx,smooth_pathy,l

import numpy as np
import matplotlib.pyplot as plt
#import random
import math
import time
#inputs:
    
start_time = time.time()
    
n = 500
r = 2
q_start = [0,0]
q_goal = [35,0]

#obstacles
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

#create the obstacle, the beast
o1 = np.array([vert_W01,vert_W02,vert_W03,vert_W04,vert_W05,vert_W06, vert_W07,vert_W08,vert_W09])


#C-space Boundaries
y_min = -5
y_max = 5

x_min = -5
x_max = 35


x,y = Sample(x_min,x_max,y_min,y_max,n)

#append start and goal to your list of points as the first and last point
x= np.insert(x,0,q_start[0])
y= np.insert(y,0,q_start[1])
x= np.append(x,q_goal[0])
y= np.append(y,q_goal[1])


x_safe,y_safe = IsSampleCollisionFree(x,y,o1)

#[ind1,ind2,dist] 
neighbors = GenerateLocalPath(x_safe,y_safe,r)
#ind1 indicates parent, ind 2 indicates neighbor, dist is distance to point

paths_new = IsPathCollisionFree(neighbors,x_safe,y_safe,o1)


paths_new = np.delete(paths_new,np.where(paths_new[:,2] == 0),axis = 0)
        

#for i in range(len(paths_new)):
 #       along_x = np.linspace(x_safe[int(paths_new[i][0])],x_safe[int(paths_new[i][1])],10)
   #     along_y = np.linspace(y_safe[int(paths_new[i][0])],y_safe[int(paths_new[i][1])],10)
  #      plt.plot(along_x,along_y)
    #    plt.plot(vert_W01[:,0],vert_W01[:,1])
     #   plt.plot(vert_W02[:,0],vert_W02[:,1])
      #  plt.plot(vert_W03[:,0],vert_W03[:,1])
       # plt.plot(vert_W04[:,0],vert_W04[:,1])
        #plt.plot(vert_W05[:,0],vert_W05[:,1])
   #     plt.plot(vert_W06[:,0],vert_W06[:,1])
    #    plt.plot(vert_W07[:,0],vert_W07[:,1])
     #   plt.plot(vert_W08[:,0],vert_W08[:,1])
      #  plt.plot(vert_W09[:,0],vert_W09[:,1])



#paths new is now my graph

#create an initialized open list, ths is the priority que
O = np.array([])
O = np.append(O,0) #add the heuristic?

O_cost = np.array([])
#create an initialized closed list - nothing
C = np.array([])
C = np.append(C,0)


#while O.size != 0:
    

f_plus = 0 #the value from start node to new node
iteration = 0

f_path = np.array([])
nbest = q_start
i_best = np.array([])

OG = paths_new
parent = np.array([])
child = np.array([])
take_path = np.array([])

#this works to find the path cost
parent_val = 0
nbest_path = 0
leave = 12

while nbest != q_goal:
    
    
    for i in range(len(paths_new)):
        if x_safe[int(paths_new[i][0])] == nbest[0] and y_safe[int(paths_new[i][0])] == nbest[1]:
            O_costp = paths_new[i][2] + f_plus
            Op = paths_new[i][1] # this is the ind, might change?
            f_oof = O_costp 
            O_cost = np.append(O_cost,O_costp)
            O = np.append(O,Op)
            f_path = np.append(f_path,f_oof)  
            child = np.append(child,paths_new[i][1])
            parent = np.append(parent,parent_val)
            
            
        
    t = int(len(parent)/(iteration+1))
    parent = parent.reshape(t,int(iteration+1))  
    #a = np.vstack((parent,child))
    #a = a.transpose()
            
    if 0 in O and iteration == 0:
        O = np.delete(O,0)
        
    if O_cost.size > 0:   
        low = np.min(O_cost)
    if O_cost.size == 0:
        leave = 13
        
    if leave == 13:
        valid_solution = 0
        print("No paths found")
        break
        
        
    ind = np.where(O_cost == low )[0][0]
    node = int(O[ind]) #indices of n_best
    
    while node in C: #this is to make sure the algorithm does not cycle, this will probably indicate if there is no path, when O is empty
        if O_cost.size>1:
            O = np.delete(O,ind)
            O_cost = np.delete(O_cost,ind)
            f_path = np.delete(f_path,ind)
            parent = np.delete(parent,ind,axis = 0)
            child = np.delete(child,ind)
            low = np.min(O_cost)
            ind = np.where(O_cost == low )[0][0]
            node = int(O[ind])
        else:
            leave = 13
            break
            
        
        
    if leave == 13:
        print("No Paths found")
        valid_solution = 0
        break
    
    a = np.array([child[ind]])
    b = parent[ind]
    parent_val = np.concatenate([b,a])#a[np.where(a[:,-1] == node)[0]]
        
    if f_path.size > 1:
        f_plus = f_path[ind]
    else:
        f_plus = f_oof
    
    C = np.append(C,node)
    O = np.delete(O,ind)
    
    nbest = [x_safe[node],y_safe[node]]
    
    if nbest != q_goal:
        O_cost = np.delete(O_cost,ind)
        parent = np.delete(parent,ind,axis = 0)
        child = np.delete(child,ind)

       # t = int(len(parent)/(iteration+1))
#        sus =  parent.reshape(t,int(iteration+1)) 
        parent = np.hstack((parent,np.ones([len(parent),1])))
    #parent = parent.transpose()
        for k in range(len(parent)):
            parent[k][-1] = parent[k][-1]*parent[k][-2]
            
    else:
        Final_Cost = O_cost[ind]
        valid_solution = 1
        print("Least Cost", Final_Cost)
        #print("Path",parent_val)
        
    


    f_path = np.delete(f_path,ind)
        
    iteration += 1
    if nbest == q_goal:
        print("Number of iterations", iteration)
        
        

x_path = np.array([])  
y_path = np.array([])      
tracking = np.array([])  
for ew in range(len(parent_val)):
    if parent_val[ew] not in tracking:
        x_path = np.append(x_path,x_safe[int(parent_val[ew])])
        y_path = np.append(y_path,y_safe[int(parent_val[ew])])
        tracking = np.append(tracking,parent_val[ew])
        
if valid_solution ==1:    
    smooth_pathx,smooth_pathy,l = pathsmooth(x_path,y_path,o1)  
    Final_Cost = l
    print(l) 
        
        

#plt.scatter(x_safe,y_safe)
plt.plot(vert_W01[:,0],vert_W01[:,1],linewidth = .75)
plt.plot(vert_W02[:,0],vert_W02[:,1],linewidth = .75)
plt.plot(vert_W03[:,0],vert_W03[:,1],linewidth = .75)
plt.plot(vert_W04[:,0],vert_W04[:,1],linewidth = .75)
plt.plot(vert_W05[:,0],vert_W05[:,1],linewidth = .75)
plt.plot(vert_W06[:,0],vert_W06[:,1],linewidth = .75)
plt.plot(vert_W07[:,0],vert_W07[:,1],linewidth = .75)
plt.plot(vert_W08[:,0],vert_W08[:,1],linewidth = .75)
plt.plot(vert_W09[:,0],vert_W09[:,1],linewidth = .75)
plt.scatter(q_goal[0],q_goal[1])
plt.scatter(q_start[0],q_start[1])
plt.plot(x_path,y_path,linewidth = .75)
plt.plot(smooth_pathx,smooth_pathy,linewidth = .75,color = 'red')
plt.title(f"Path Length is {Final_Cost}")

end_time = time.time()

total_time = end_time - start_time
print("Total time in seconds", total_time)
