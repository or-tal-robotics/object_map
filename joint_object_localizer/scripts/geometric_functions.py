#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from scipy.optimize import differential_evolution
from sklearn.neighbors import NearestNeighbors
from tf.transformations import quaternion_from_euler


def _Init_Ellipse(theta):
        
    x0 = theta[0]
    y0 = theta[1]
    phi = theta[2]
    a = theta[3]
    b = theta[4]
    alpha = np.linspace(np.pi , 2 * np.pi, 26)
    x1 = a * np.cos(alpha)
    y1 = b * np.sin(alpha)
    x = x1 * np.cos(phi) - y1*np.sin(phi) + x0
    y = y1 * np.cos(phi) + x1*np.sin(phi) + y0
    Ellipse_vector = np.array([x,y])
    return Ellipse_vector

def _Initializing_half_Rectangle(theta):

    x0 = theta[0]
    y0 = theta[1]
    phi = theta[2]
    a = theta[3]
    b = theta[4]

    if phi > 0 and phi < np.pi or phi < 0 and phi > -np.pi:
            
        x1 = -a/2 * np.ones(13)
        y1 = np.linspace(-b/2 , b/2 , 13)
        x2 = np.linspace(-a/2 , a/2 , 13)
        y2 = -b/2 *  np.ones(13)
        x3 = np.concatenate((x1,x2) , axis = None)
        y3 = np.concatenate((y1,y2) , axis = None)
        x = x3 * np.cos(phi) - y3 * np.sin(phi) + x0
        y = y3 * np.cos(phi) + x3 * np.sin(phi) + y0
        Rectangle_vector = np.array([x,y]).T
            
    elif phi == 0 or phi == np.pi/2 or phi == -np.pi/2:
        n = y0 - a/2
        x1 = x0 - a/2
        x2 = x0 + a/2
        x_v = np.linspace(x1,x2,26)
        y_v = np.ones(26) * n
        Rectangle_vector = np.array([x_v,y_v]).T
            
    else:
        x_v = np.linspace(0,100,26)
        y_v = np.ones(26) * 9999
        Rectangle_vector = np.array([x_v,y_v]).T
            
            
    return Rectangle_vector

def quaternion_to_euler(x, y, z, w):
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return [yaw, pitch, roll]


# R - robot, c - old center
def New_Ce_array(x_c,y_c,x_R,y_R,yaw):
    x = x_R +  x_c*np.cos(yaw) - y_c*np.sin(yaw)
    y = y_R +  y_c*np.cos(yaw) + x_c*np.sin(yaw)
   
    # Return the real pose of the point after changing it from the view of the laser:
    return np.array([x,y])


# Calculating the distance between two points:
def _distance(pt_1, pt_2):
    pt_1 = np.array((pt_1[0], pt_1[1]))
    pt_2 = np.array((pt_2[0], pt_2[1]))
    return np.linalg.norm(pt_1-pt_2)

def closest_node(node, nodes):
    
    i = -1
    j = 0
    dist = 9999999
    for n in nodes:
        
        if _distance(node, n) <= dist:
            dist = _distance(node, n)
            
            j = i
        i += 1
    return [dist , j]