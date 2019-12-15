#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from scipy.stats import multivariate_normal as mn
from scipy.optimize import differential_evolution

global Object_cls_list
# YAMLs:
Object_cls_list = np.array(rospy.get_param('/Array/object_list'))



def _Rearrange_SSD_probability(Old_SSD_Probabilities):
    Old_SSD_Probabilities = np.array(Old_SSD_Probabilities)
    global Object_cls_list
    New_SSD_Probabilities = np.zeros(20)
    Normalization_Factor = np.sum(Old_SSD_Probabilities[Object_cls_list-1])
    for ii in (Object_cls_list-1):
        New_SSD_Probabilities[ii] = Old_SSD_Probabilities[ii]/Normalization_Factor
    
    return New_SSD_Probabilities

    

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

def GMM_Likelihood(mu,sigma,Z):
    n = mu.shape[1]
    L = np.zeros(Z.shape[0])
    for i in range(0,n):
        L += np.array(mn.pdf(Z,mean = mu[:,i],cov = sigma))
    return np.sum(L/n)

def GMM_Prior(mu,sigma,Z):
    
    
    L = np.array(mn.pdf(Z,mean = mu,cov = sigma))
    
    return np.sum(L)

# R - robot, c - old center
def New_Ce_array(x_c,y_c,x_R,y_R,yaw,correction):
    x = x_R +  x_c*np.cos(yaw) - y_c*np.sin(yaw) + correction*np.cos(yaw)
    y = y_R +  y_c*np.cos(yaw) + x_c*np.sin(yaw) + correction*np.sin(yaw)
   
    # Return the real pose of the point after changing it from the view of the laser:
    return np.array([x,y])

def _Init_Ellipse(theta):
        
        x0 = theta[0]
        y0 = theta[1]
        phi = theta[2]
        a = theta[3]
        b = theta[4]
        alpha = np.linspace(np.pi , 2 * np.pi, 5)
        x1 = a * np.cos(alpha)
        y1 = b * np.sin(alpha)
        x = x1 * np.cos(phi) - y1*np.sin(phi) + x0
        y = y1 * np.cos(phi) + x1*np.sin(phi) + y0
        Ellipse_vector = np.array([x,y])
        return Ellipse_vector

def _Distances_from_center_of_ellipse(theta,Z):

    # Initializing and identifying variables:
    Distance_Vector = []
    x_c = theta[0]
    y_c = theta[1]
    angle = theta[2]
    a = theta[3]
    b = theta[4]

    # Making a list of distances from the center of the ellipse:
    for z in Z:

        R_x = ((z[0]-x_c)*np.cos(angle) + (z[1]-y_c)*np.sin(angle))/(a)
        R_y = ((z[0]-x_c)*np.sin(angle) - (z[1]-y_c)*np.cos(angle))/(b)
        Distance_Vector.append(R_x**2 + R_y**2)

    return np.array(Distance_Vector)

def _Initializing_half_Rectangle(theta):

    x0 = theta[0]
    y0 = theta[1]
    phi = theta[2]
    a = theta[3]
    b = theta[4]

    if phi > 0 and phi < np.pi or phi < 0 and phi > -np.pi:
        
        b_a = 7*int(b/(a+b))
        x1 = -a/2 * np.ones(b_a)
        y1 = np.linspace(-b/2 , b/2 , b_a)
        x2 = np.linspace(-a/2 , a/2 , 7 -b_a)
        y2 = -b/2 *  np.ones(7-b_a)
        x3 = np.concatenate((x1,x2) , axis = None)
        y3 = np.concatenate((y1,y2) , axis = None)
        x = x3 * np.cos(phi) - y3 * np.sin(phi) + x0
        y = y3 * np.cos(phi) + x3 * np.sin(phi) + y0
        Rectangle_vector = np.array([x,y]).T
            
    elif phi == 0 or phi == np.pi/2 or phi == -np.pi/2:
        n = y0 - a/2
        x1 = x0 - a/2
        x2 = x0 + a/2
        x_v = np.linspace(x1,x2,7)
        y_v = np.ones(7) * n
        Rectangle_vector = np.array([x_v,y_v]).T
            
    else:
        x_v = np.linspace(0,100,7)
        y_v = np.ones(7) * 9999
        Rectangle_vector = np.array([x_v,y_v]).T
            
            
    return Rectangle_vector

class Likelihood():

    def __init__(self,class_number=[],Z=[],SSD_probability=[]):
        
        self.class_number = class_number
        self.Z = Z
        self.Probability = _Rearrange_SSD_probability(SSD_probability)
    
    # Likelihood functions:
    def probability_for_circle(self,theta):
        
        R = theta[2]

        sigma = 0.0005
        sigma_prior = rospy.get_param('/object_list/o'+str(self.class_number) + '/cov')
        r_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/r')

        r_c = ( (theta[0]-self.Z[:,0])**2 + (theta[1]-self.Z[:,1])**2 )**0.5
        
        L = GMM_Prior(R,sigma,r_c)
        Prior = GMM_Prior(R,sigma_prior,r_mean)
        
        return -Prior * L * self.Probability[self.class_number-1]
        
    

    def probability_for_Rectangle(self,theta):

            a = theta[3]
            b = theta[4]
            
            Rectangle_vector = _Initializing_half_Rectangle(theta).T       
            
            sigma = np.array([[0.002,0],[0,0.002]])
            sigma_prior = np.array(rospy.get_param('/object_list/o'+str(self.class_number) + '/cov'))
            L = GMM_Likelihood(Rectangle_vector,sigma,self.Z)
            

            a_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/a')
            b_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/b')
            Prior = GMM_Prior(np.array([a,b]),sigma_prior,np.array([a_mean,b_mean]))

            return -Prior* L * self.Probability[self.class_number-1]
            

            
    def probability_for_Ellipse(self,theta):

            a = theta[3]
            b = theta[4]
            #Ellipse_vector = _Init_Ellipse(theta)
            Ellipse_vector = _Distances_from_center_of_ellipse(theta,self.Z)
            sigma = 0.05
            sigma_prior = np.array(rospy.get_param('/object_list/o'+str(self.class_number) + '/cov'))
            L = GMM_Prior(1,sigma,Ellipse_vector)
            a_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/a')
            b_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/b')
            
            Prior = GMM_Prior(np.array([a,b]),sigma_prior,np.array([a_mean,b_mean]))

            return -Prior * L * self.Probability[self.class_number-1]

            





        
