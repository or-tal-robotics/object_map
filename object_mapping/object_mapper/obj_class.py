#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np


# updating location and sizes:
def Theta_updater(x_old,x_new,
                y_old,y_new,
                r_old,r_new,
                a_old,a_new,
                b_old,b_new,
                phi_old,phi_new):
                '''Update exsisting object values.
                Return updated Mi'''
                x = (x_old + x_new)/2
                y = (y_old + y_new)/2
                r = (r_old + r_new)/2
                a = (a_old + a_new)/2
                b = (b_old + b_new)/2
                phi = (phi_old + phi_new)/2

                return x , y , r , a , b , phi
    

# Updating the Probabilities of every object with consideration for the old ones:
def Updated_Probabilities_and_Cls(old_probability,new_probability,new_cls):
    '''Update the probability and the class of founded object.
    Returns [updated probability , the updated class number] '''
    if len(np.array(old_probability)) < 1:
        new_cls = np.argmax(new_probability)+1
        updated_prob = new_probability
    else:
        updated_prob = 0.5 * (np.array(old_probability) + np.array(new_probability))
        sumU = np.sum(updated_prob)
        updated_prob = updated_prob / sumU
        new_cls = np.argmax(updated_prob)+1
    return updated_prob, new_cls

# Calculating the distance between two points:
def _distance(pt_1, pt_2):
    '''Calculate distance betwenn two points.
    Returns distance value'''
    pt_1 = np.array((pt_1[0,0], pt_1[0,1]))
    pt_2 = np.array((pt_2[0], pt_2[1]))
    return np.linalg.norm(pt_1-pt_2)

def closest_node(node, nodes):
    '''Finding the closest point to a given point.
    Return [distance , closest point index] '''
    i = 0
    j = 0
    dist = 9999999
    for n in nodes:
        if _distance(node, n) <= dist:
            dist = _distance(node, n)
            j = i
        i += 1
    return [dist , j]

def Search_Radius(r,a,b):
    '''Return the maximum value from a,b,r.
    Fo example, if object is circle, it will return the value of r'''
    return np.amax([r,a,b])

class Object_Map_cls():


    def __init__(self,x_center=0,y_center=0,r=0,a=0,b=0,angle=0,cls_num=0,prob_distribution=[],viewed_number=0):

        self.x_center = x_center
        self.y_center = y_center
        self.r = r
        self.a = a
        self.b = b
        self.angle = angle
        self.cls_num = cls_num
        self.prob_distribution = prob_distribution

