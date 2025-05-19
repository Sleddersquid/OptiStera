# -*- coding: utf-8 -*-
"""
Created on Mon Jan 20 13:43:47 2025

@author: magnu
"""

'''
The following code is based on the formulas given in 
K. Liu, J. Fitzgerald, and F. Lewis, “Kinematic analysis of a stewart platform
manipulator,” IEEE Transactions on Industrial Electronics, vol. 40, no. 2,
pp. 282–293, Apr. 1993, issn: 1557-9948. doi: 10.1109/41.222651.
'''

import numpy as np

def XYZval(Px,Py,Pz,a,alpha,beta,gamma):   #Function to calculate values needed for the calculation of actuator lengths.
    Xt1 = Px + (a/np.sqrt(3))*(np.sin(alpha)*np.sin(beta)*np.sin(gamma + 60*(np.pi/180)) + np.cos(beta)*np.cos(gamma + 60*(np.pi/180)))
    Xt2 = Px - (a/np.sqrt(3))*(np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(beta)*np.cos(gamma))
    Xt3 = Px + (a/np.sqrt(3))*(np.sin(alpha)*np.sin(beta)*np.sin(gamma -60*(np.pi/180)) + np.cos(beta)*np.cos(gamma - 60*(np.pi/180)))
    
    Yt1 = Py + (a/np.sqrt(3))*(np.cos(alpha)*np.sin(gamma + 60*(np.pi/180)))
    Yt2 = Py - (a/np.sqrt(3))*(np.cos(alpha)*np.sin(gamma))
    Yt3 = Py + (a/np.sqrt(3))*(np.cos(alpha)*np.sin(gamma -60*(np.pi/180)))
    
    Zt1 = Pz + (a/np.sqrt(3))*(np.sin(alpha)*np.cos(beta)*np.sin(gamma + 60*(np.pi/180)) - np.sin(beta)*np.cos(gamma + 60*(np.pi/180)))
    Zt2 = Pz - (a/np.sqrt(3))*(np.sin(alpha)*np.cos(beta)*np.sin(gamma) - np.sin(beta)*np.cos(gamma))
    Zt3 = Pz + (a/np.sqrt(3))*(np.sin(alpha)*np.cos(beta)*np.sin(gamma - 60*(np.pi/180))- np.sin(beta)*np.cos(gamma - 60*(np.pi/180)))
    
    return(Xt1, Xt2, Xt3, Yt1, Yt2, Yt3, Zt1, Zt2, Zt3) #Returns the values needed to calculate actuator lengths. 

def Length(Xt1, Xt2, Xt3, Yt1, Yt2, Yt3, Zt1, Zt2, Zt3, d, b):   #Function to calculate actuator lengths. 
    L1 = np.sqrt((Xt1 - (d/(2*np.sqrt(3))) - (b/np.sqrt(3)))**2 + (Yt1 - (d/2))**2 + Zt1**2)
    L2 = np.sqrt((Xt1 - (d/(2*np.sqrt(3))) + (b/(2*np.sqrt(3))))**2 + (Yt1 - (d/2) -(b/2))**2 + Zt1**2)
    L3 = np.sqrt((Xt2 + (d/np.sqrt(3)) + (b/(2*np.sqrt(3))))**2 + (Yt2 - (b/2))**2 + Zt2**2)
    L4 = np.sqrt((Xt2 + (d/np.sqrt(3)) + (b/(2*np.sqrt(3))))**2 + (Yt2 + (b/2))**2 + Zt2**2)
    L5 = np.sqrt((Xt3 - (d/(2*np.sqrt(3))) +(b/(2*np.sqrt(3))))**2 + (Yt3 + (b/2) + (d/2))**2 + Zt3**2)
    L6 = np.sqrt((Xt3 - (d/(2*np.sqrt(3))) - (b/np.sqrt(3)))**2 + (Yt3 + (d/2))**2 + Zt3**2)
    
    return(L1, L2, L3, L4, L5, L6)  #Returns the actuator lengths. 

Px = 0  #Zero translatory movement in x-direction.
Py = 0  #Zero translatory movement in y-direction.
Pz = 300    #Height from the origin of the base platform to the origin of the mobile platform. 

a = 180 #Length in mm of the sides of the mobile platform.

b = 300 #Length in mm of the longest side of the base platform.
d = 70  #Length in mm of the shortest side of the base platform. 

alpha = -30*(np.pi/180) #Degrees of tilt of the mobile platform around the x-axis.
beta = 0*(np.pi/180)    #Degrees of tilt of the mobile platform around the y-axis.
gamma = 0*(np.pi/180)   #Degrees of tilt of the mobile platform around the z-axis. 

Xt1, Xt2, Xt3, Yt1, Yt2, Yt3, Zt1, Zt2, Zt3 = XYZval(Px, Py, Pz, a, alpha, beta, gamma) #Calling on the function for values needed for the calculation of actuator lengths. 

L1, L2, L3, L4, L5, L6 = Length(Xt1, Xt2, Xt3, Yt1, Yt2, Yt3, Zt1, Zt2, Zt3, d, b)  #Calling on the function for the calculation of actuator lengths. 

print('L1: ', round(L1,2), 'L2: ', round(L2, 2), 'L3: ', round(L3, 2), 'L4: ', round(L4, 2), 'L5: ', round(L5, 2), 'L6: ', round(L6, 2))   #Printing the actuator lengths. 




    