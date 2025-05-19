# -*- coding: utf-8 -*-
"""
Created on Sat Jan 25 18:41:59 2025

@author: magnu
"""

'''
The following code is based on the formulas given in 
G. Cui, M. Sun, L. Yan, H. Hou, and H. Zhang, “Kinematic reliability solution of
3-ups-pu parallel mechanism based on monte carlo simulation,”
The Open Mechanical Engineering Journal, vol. 9, pp. 324–332, May 2015.
doi: 10.2174/1874155X01509010324. 
'''

import numpy as np 

'''
Function using tilt along x- and y-axis, the height of  
the origin of the mobile platform and the radius of the lower and upper platform
to calculate actuator lengths of a 3UPS-PU PM:
'''
def length(theta, psi, h, r_a, r_b,): 
    l_1 = (np.cos(theta)*r_a - r_b)**2 + (-np.sin(theta)*r_a + h)**2
    
    l_2 = ((-1/2)*np.cos(theta)*r_a+(1/3)*np.sin(theta)*np.sin(psi)*np.sqrt(3)*r_a + (1/2)*r_b)**2 \
            +((1/2)*np.cos(psi)*np.sqrt(3)*r_a-(1/2)*np.sqrt(3)*r_b)**2 \
            +((1/2)*np.sin(theta)*r_a+(1/2)*np.cos(theta)*np.sin(psi)*np.sqrt(3)*r_a + h)**2
            
    l_3 = (-(1/2)*np.cos(theta)*r_a-(1/2)*np.sin(theta)*np.sin(psi)*np.sqrt(3)*r_a + (1/2)*r_b)**2 \
            +(-(1/2)*np.cos(psi)*np.sqrt(3)*r_a + (1/2)*np.sqrt(3)*r_b)**2 \
            +((1/2)*np.sin(theta)*r_a - (1/2)*np.cos(theta)*np.sin(psi)*np.sqrt(3)*r_a + h)**2
            
    L_1 = np.sqrt(l_1)  
    L_2 = np.sqrt(l_2)
    L_3 = np.sqrt(l_3)
            
    return(L_1, L_2, L_3)   #Return the actuator lengths 

theta = 0            #Rotation (in degrees) around y-axis.
psi = 10*(np.pi/180) #Rotation (in degrees) around x-axis.

h = 315 #Height of from the base platform up to the origin of the mobile platform 

r_b = 100 #Radius of the base platform from to origin to the actuator mount. 
r_a = 80  #Radius of the mobile platform from the orign to the actuator mount. 

L_1, L_2, L_3 = length(theta, psi, h, r_a, r_b) #Calling on the function for actuator lengths 

print('L_1: ', round(L_1, 2), 'L_2: ', round(L_2, 2), 'L_3: ', round(L_3, 2)) #Printing actuator lengths



