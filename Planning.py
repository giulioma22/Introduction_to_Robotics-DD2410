#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Giulio Martinelli
# 19960222T351
# giulioma@kth.se

from dubins import Car
import math
import numpy as np
import numpy.linalg as la
import random


obs = None
MAX_STEP = 150
goal = None
dt = 0
randArea = None


class Node():

    def __init__(self, x, y, theta):
        self.x = x 
        self.y = y
        self.theta = theta
        self.parent = None
        self.ctrl = None        
        
class Ctrl():
    
    def __init__(self, phi, steps):
        self.phi = phi
        self.steps = steps 

def RRT(car, start, goal):
    
    RRT.nodes = [start]
    x, y, theta = None, None, None
    xn, yn, thetan = [None] * MAX_STEP, [None] * MAX_STEP, [None] * MAX_STEP

    
    while True : 
        if random.randint(0, 100) > RRT.goalSampleRate: 
            rnd = [random.uniform(randArea[0], randArea[1]), random.uniform(randArea[2], randArea[3])]
        else : 
            rnd = [goal.x, goal.y]
        
        rndNode = Node(rnd[0], rnd[1], 0)
        nearestNode = RRT.nodes[GetNearestNode(RRT.nodes, rnd)]
        
        phi = steerAng(nearestNode, rnd)

        x, y, theta = nearestNode.x, nearestNode.y, nearestNode.theta

        for i in range(MAX_STEP) :
            xn[i], yn[i], thetan[i] = car.step(x, y, theta, phi)
            x, y, theta = xn[i], yn[i], thetan[i] 
            
            finished =  Arrived([xn[i], yn[i]])
                
            if((not Safe(xn[i], yn[i])) or finished):
                break  
            
        if finished :  
            goal.parent = nearestNode
            goal.ctrl = Ctrl(phi, i)
            print("Goal!")
            break
            
        
        if i != MAX_STEP -1 : 
            i = int((i)/3) 
            
        
        if i == 0 :
            continue
            
        
        newNode = Node(xn[i], yn[i], thetan[i])
        newNode.parent = nearestNode
        newNode.ctrl = Ctrl(phi, i) 
        RRT.nodes.append(newNode)

    
    ctrls = [goal.ctrl]
    path = [goal]
    while path[-1].parent is not None:
        ctrls.append(path[-1].parent.ctrl)
        path.append(path[-1].parent)
    path.pop()
    ctrls.pop()
        
    return list(reversed(ctrls))
        
        
def GetNearestNode(nodes, rnd):
    
    distances = [(node.x-rnd[0])**2+(node.y-rnd[1])**2 for node in nodes]
    return distances.index(min(distances))


def Safe(x, y):
    for (ox, oy, size) in obs:
        dx = ox - x
        dy = oy - y
        d = math.sqrt(dx * dx + dy * dy)
        if d <= size+ 0.1:
            return False  # collision
    
    inbounds = x > randArea[0] and x < randArea[1]-0.1 and y > randArea[2]+0.1 and y < randArea[3]-0.1
    return inbounds

def Arrived(p):
    return ((goal.x-p[0])**2 + (goal.y-p[1])**2)**0.5 < 0.2

def steerAng(nearestNode, rnd):
    v = [math.cos(nearestNode.theta), math.sin(nearestNode.theta), 0]
    rg = [rnd[0]-nearestNode.x, rnd[1]-nearestNode.y, 0]
    prodNorm = la.norm(v)*la.norm(rg)
    cosPhi = np.dot(v, rg)/prodNorm
    sinPhi = np.cross(v, rg)[2]/prodNorm
    phi = math.atan2(sinPhi, cosPhi)

    if phi > math.pi/4:
        phi = math.pi/4
    elif phi < - math.pi/4:
        phi = - math.pi/4
    return phi


def solution(car):

    global obs, randArea, goal, dt

    obs = car.obs
    randArea=[car.xlb, car.xub, car.ylb, car.yub]
    goal = Node(car.xt, car.yt, 0)
    start = Node(car.x0, car.y0, 0)
    start.ctrl = Ctrl(0,0)
    dt = car.dt
    
    controls = RRT(car, start, goal)
    times = [0]
    finalControls = []
    i= 1

    for ctrl in controls : 
        times.append(times[-1] + ctrl.steps * dt)
        finalControls.append(ctrl.phi)
        i+=1

    return finalControls, times
RRT.goalSampleRate = 15  