
#With rotation
#assuming the additional parameter is theta

def findanglebad(vert,theta_r): #updated for rotating robot
    
    #input: Vert: the vertices of the robot 
    #input: theta_r: The rotated ange of the robot in radians
    angles = np.array([])
    i = 0
    for i in range(len(vert)-1):
        V = np.array((vert[i+1]-vert[i])/np.linalg.norm(vert[i+1]-vert[i])) 
        unit = np.array([1,0])
        theta = np.arccos(np.dot(V,unit)/(np.linalg.norm(V)*np.linalg.norm(unit))) + theta_r #rads
        if V[1] == -1 and V[0] == 0:
            theta = 270*np.pi/180 + theta_r #rads
        if i == 2 and theta < 180*np.pi/180:
            unit = np.array([-1,0])
            theta = np.arccos(np.dot(V,unit)/(np.linalg.norm(V)*np.linalg.norm(unit))) + np.pi
            
        angles = np.append(angles,theta)
    #vert: input list of vertices
    
    return angles

def findangle(vert):
    angles = np.array([])
    i = 0
    for i in range(len(vert)-1):
        V = np.array((vert[i+1]-vert[i])/np.linalg.norm(vert[i+1]-vert[i])) 
        unit = np.array([1,0])
        theta = np.arccos(np.dot(V,unit)/(np.linalg.norm(V)*np.linalg.norm(unit)))*180/np.pi
        if V[1] == -1 and V[0] == 0:
            theta = 270 #degrees
        if i == 2 and theta < 180:
            unit = np.array([-1,0])
            theta = np.arccos(np.dot(V,unit)/(np.linalg.norm(V)*np.linalg.norm(unit)))*180/np.pi + 180
        
            
        angles = np.append(angles,theta)
    #vert: input list of vertices
    
    return angles

def Rotated_Verts(verts,theta):
    #input: vert: vertices of the object
    #input: theta: the theta value it is rotated by 
    new_verts = np.zeros(shape=(4,2))
    
    for i in range(len(verts)):
        if i == 0 or i == 3:
            new_verts[i] = verts[i]
        else:
            xp = verts[0,0] + (verts[i,0]-verts[0,0])*np.cos(theta) - (verts[i,1]-verts[0,1])*np.sin(theta)
            yp = verts[0,1] + (verts[i,0]-verts[0,0])*np.sin(theta) + (verts[i,1]-verts[0,1])*np.cos(theta)
            new_verts[i] = [xp,yp]
    
    return new_verts

#determind C Space Obstacles with no translation
#vertices are counterclockwise order v1 and w1 have smallest y value
import numpy as np
import matplotlib.pyplot as plt

theta = np.zeros(12)
k = 0
for k in range(12):
    theta[k] = 2*np.pi*(k/11)
    
#create the 12 points for theta

#deal with vertices later
b1 = [0,0]
b2 = [1,2]
b3 = [0,2]

a1 = [-1,-2] #same shape are obstacle
a2 = [0,-2]
a3 = [0,0]
#a1 = [-2,-2] #global points?
#a2 = [-1,0]
#a3 = [-2,0]

v1 = np.array([b1,b2,b3,b1]) #obstacle verts
w1 = np.array([a1,a2,a3,a1]) # robot verts


m = len(w1)-1 #number of verticles on obstacle
n = len(v1)-1 #number of obstacles on robot




C_space = np.zeros(shape=(12,6,2))
#C_space[:,0,0] = theta

for l in range(len(theta)):
    
    counter = 0
    i = 0
    j = 0

    new_w = Rotated_Verts(w1,theta[l])
    
    while (i < n-1) or (j < m-1):
        
    
        o_ang = findangle(v1) #angles of obstacle
        r_ang = findangle(w1) #angles of the robot, affected by rotation
        
        #rotating the robots points in its local frame- that is all


        if r_ang[i] < o_ang[j]:
            vert = new_w[i] + v1[j]
            i += 1
        elif r_ang[i] > o_ang[j]:
            vert = new_w[i] + v1[j]
            j +=1
        else:
            vert = new_w[i] + v1[j]
            j+=1
            i+=1
        
        C_space[l,counter] = vert
        counter += 1
        
        if i == n-1 and j == m-1:
            vert = w1[i] + v1[j]
            C_space[l,counter] = vert
            C_space[l,5] = C_space[l,0]
        

#C_space = np.append(C_space,C_space[0:2])    
#t = len(C_space)/2

#C_space = C_space.reshape(int(t),2)
ax = plt.axes(projection='3d')
ax.set_xlabel('x') 
ax.set_ylabel('y')
ax.set_zlabel('Theta')
ax.view_init(20, 80) #fix the view of the plot here
for q in range(12):
    ax.plot(C_space[q,:,0], C_space[q,:,1],np.ones(6)*theta[q])
