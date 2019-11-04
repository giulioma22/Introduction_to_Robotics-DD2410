#! /usr/bin/env python3

"""
    # {Giulio Martinelli}
    # {giulioma@kth.se}
"""

from math import atan2, sqrt, sin, cos
import numpy as np

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]

    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    c2 = ((x-l0)**2 + y**2 - l1**2 - l2**2) / (2*l1*l2)
    s2 = sqrt(1 - c2**2)
    q2 = atan2(s2, c2)

    q1 = atan2(y, x-l0) - atan2(l2*s2, l1+l2*c2)

    q = [q1, q2, z]

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions

    H = 0.311
    L = 0.4
    M = 0.39
    N = 0.078
    
    while(True):
    
        A1 = [[cos(q[0]),0,sin(q[0]),0],[sin(q[0]),0,-cos(q[0]),0],[0,1,0,0],[0,0,0,1]]
        A2 = [[cos(q[1]),0,-sin(q[1]),0],[sin(q[1]),0,cos(q[1]),0],[0,-1,0,0],[0,0,0,1]]
        A3 = [[cos(q[2]),0,-sin(q[2]),0],[sin(q[2]),0,cos(q[2]),0],[0,-1,0,L],[0,0,0,1]]
        A4 = [[cos(q[3]),0,sin(q[3]),0],[sin(q[3]),0,-cos(q[3]),0],[0,1,0,0],[0,0,0,1]]
        A5 = [[cos(q[4]),0,sin(q[4]),0],[sin(q[4]),0,-cos(q[4]),0],[0,1,0,M],[0,0,0,1]]
        A6 = [[cos(q[5]),0,-sin(q[5]),0],[sin(q[5]),0,cos(q[5]),0],[0,-1,0,0],[0,0,0,1]]
        A7 = [[cos(q[6]),-sin(q[6]),0,0],[sin(q[6]),cos(q[6]),0,0],[0,0,1,0],[0,0,0,1]]

        T2 = np.dot(A1,A2)
        T3 = np.dot(T2,A3)
        T4 = np.dot(T3,A4)
        T5 = np.dot(T4,A5)
        T6 = np.dot(T5,A6)

        T = np.dot(T6,A7)
    
        z0 = [0,0,1]
        z1 = np.dot(A1,[0,0,1,0])[0:3]
        z2 = np.dot(T2,[0,0,1,0])[0:3]
        z3 = np.dot(T3,[0,0,1,0])[0:3]
        z4 = np.dot(T4,[0,0,1,0])[0:3]
        z5 = np.dot(T5,[0,0,1,0])[0:3]
        z6 = np.dot(T6,[0,0,1,0])[0:3]
        
        p0 = [0,0,0]
        p1 = np.dot(A1,[0,0,0,1])[0:3]
        p2 = np.dot(T2,[0,0,0,1])[0:3]
        p3 = np.dot(T3,[0,0,0,1])[0:3]
        p4 = np.dot(T4,[0,0,0,1])[0:3]
        p5 = np.dot(T5,[0,0,0,1])[0:3]
        p6 = np.dot(T6,[0,0,0,1])[0:3]
        
        P = np.dot(T,[0,0,0,1])[0:3]
        
        jacobian = np.transpose([np.concatenate((np.cross(z0,P-p0), z0)), 
                    np.concatenate((np.cross(z1,P-p1), z1)), 
                    np.concatenate((np.cross(z2,P-p2), z2)), 
                    np.concatenate((np.cross(z3,P-p3), z3)), 
                    np.concatenate((np.cross(z4,P-p4), z4)), 
                    np.concatenate((np.cross(z5,P-p5), z5)), 
                    np.concatenate((np.cross(z6,P-p6), z6))])

        t_jac = np.linalg.pinv(jacobian)
       
        current_x = np.dot(T,[0, 0, N, 1])[0:3]
        current_x[2] += H
    
        desir_x = [x, y, z]
   
        error_x = current_x - desir_x

        orient_error = 1/2*(  np.cross(np.dot(R, [1, 0, 0]), np.dot(T,[1, 0, 0, 0])[0:3]) 
                            + np.cross(np.dot(R, [0, 1, 0]), np.dot(T,[0, 1, 0, 0])[0:3]) 
                            + np.cross(np.dot(R, [0, 0, 1]), np.dot(T,[0, 0, 1, 0])[0:3]))
    
        tot_error = np.concatenate((error_x, orient_error))
        
        error_theta = np.dot(t_jac, tot_error)

        if np.linalg.norm(error_theta) < 0.01:
            break
            
        q = q - error_theta
    
    return q