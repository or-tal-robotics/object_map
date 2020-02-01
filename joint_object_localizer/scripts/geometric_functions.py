#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from scipy.stats import multivariate_normal as mn
from scipy.optimize import differential_evolution

# Msgs:
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

global Object_cls_list
# YAMLs:
Object_cls_list = np.array(rospy.get_param('/Array/object_list'))

# Publisher:
Object_P_pub = rospy.Publisher('/OB_Points' , PointCloud , queue_size = 5 )

def Ranges_to_xy_plane(R,angle_min,angle_max,
                        size,angle_jumps,
                        max_dist = 2):

    '''Transfer the ranges into (x,y) dots. return array [n,2]'''
    # Initializing an inf values:
    R[np.isinf(R)] = 0

    # Initializing an inf values:
    R[np.isnan(R)] = 0

    # Checking info about the location of the object:
    Dist_check = np.array(R[angle_min:angle_max])
    Dist_check = Dist_check[Dist_check != 0]

    # If there are not enough measuremrnts:
    if len(Dist_check) < 4:
        return np.array([0,0])
    # If object is too far away:
    if np.amin(Dist_check) > max_dist:
        return np.array([0,0])
    
    print ('Close enough, starting.')

    laser_depth_observations = np.zeros((2*size + 1,2))
    P = PointCloud()
    P.points = []
    P.header.frame_id = 'laser_link'
    for i in range (0,size):
        
        if R[i] == 0:
            laser_depth_observations[i,0] = 100000
            laser_depth_observations[i,1] = 100000
            continue
        laser_depth_observations[i,0] = -R[i] * np.cos((size - i) * angle_jumps - np.pi)
        laser_depth_observations[i,1] = R[i] * np.sin((size - i) * angle_jumps - np.pi)
        
        if i < angle_max and i >= angle_min:
            xyz = Point32()
            xyz.x = laser_depth_observations[i,0]
            xyz.y = laser_depth_observations[i,1]
            xyz.z = 0
            P.points.append(xyz)

    
    for i in range (size,2*size+1):
        if R[i] == 0:
            laser_depth_observations[i,0] = 100000
            laser_depth_observations[i,1] = 100000
            continue
        laser_depth_observations[i,0] = R[i] * np.cos((i-size) * angle_jumps)
        laser_depth_observations[i,1] = R[i] * np.sin((i-size) * angle_jumps)

        if i < angle_max and i >= angle_min:
            xyz = Point32()
            xyz.x = laser_depth_observations[i,0]
            xyz.y = laser_depth_observations[i,1]
            xyz.z = 0
            
            P.points.append(xyz)
    Object_P_pub.publish(P)
    return laser_depth_observations

def _Rearrange_SSD_probability(Old_SSD_Probabilities):
    '''Return updated SSD probabilities'''
    Old_SSD_Probabilities = np.array(Old_SSD_Probabilities)
    global Object_cls_list
    New_SSD_Probabilities = np.zeros(20)
    Normalization_Factor = np.sum(Old_SSD_Probabilities[Object_cls_list-1])
    for ii in (Object_cls_list-1):
        New_SSD_Probabilities[ii] = Old_SSD_Probabilities[ii]/Normalization_Factor
    
    return New_SSD_Probabilities

def Object_height_update(height_factor,a,b,phi):
    phi = -phi
    '''Return the object height'''
    if phi < np.pi/2:
        object_x_width = a * np.cos(phi) + b * np.sin(phi)
    else:
        object_x_width = a * np.cos(np.pi - phi) + np.sin(phi-np.pi/2)
    return object_x_width * height_factor    
    
def quaternion_to_euler(x, y, z, w):
    '''Input quanterion angle.
    Return euler angles [Yaw,Pitch,Roll]'''
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
    '''Return sum of all the GMM density functions.
    Use when mu size is more than 1'''
    n = mu.shape[1]
    L = np.zeros(Z.shape[0])
    for i in range(0,n):
        L += np.array(mn.pdf(Z,mean = mu[:,i],cov = sigma))
    return np.sum(L/n)

def GMM_Prior(mu,sigma,Z):
    '''Return sum of all the GMM density functions.
    Use when mu size is 1'''
    L = np.array(mn.pdf(Z,mean = mu,cov = sigma))
    
    return np.sum(L)

# R - robot, c - old center
def New_Ce_array(x_c,y_c,x_R,y_R,yaw,correction):
    '''Rotation and translation from the robot view to map view.
    Return [x,y] array'''
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
    '''Uses rotated and translated ellipse function on the depth observations.
    Return the value on all those observations.
    (If point is on the ellipse, return value of 1 for this point) '''
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
    '''Return vector of points that will act as components of GMM for the rectangle.
    Return array of [x,y]'''
    x0 = theta[0]
    y0 = theta[1]
    phi = theta[2]
    a = theta[3]
    b = theta[4]

    if phi > 0 and phi < np.pi or phi < 0 and phi > -np.pi:
        
        b_a = int(10*b/(a+b))
        x1 = ((np.cos(phi))/(np.absolute(np.cos(phi))))*(-a/2) * np.ones(b_a)
        y1 = np.linspace(-b/2 , b/2 , b_a)
        x2 = np.linspace(-a/2 , a/2 , 10 -b_a)
        y2 = -b/2 *  np.ones(10-b_a)
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
        '''Return posterior for circle shape for a given theta'''
        
        R = theta[2]

        sigma = 0.0005
        sigma_prior = rospy.get_param('/object_list/o'+str(self.class_number) + '/cov')
        r_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/r')

        r_c = ( (theta[0]-self.Z[:,0])**2 + (theta[1]-self.Z[:,1])**2 )**0.5
        
        L = GMM_Prior(R,sigma,r_c)
        Prior = GMM_Prior(R,sigma_prior,r_mean)
        
        return -Prior * L * self.Probability[self.class_number-1]
        
    

    def probability_for_Rectangle(self,theta):
            '''Return posterior for rectangle shape for a given theta'''
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
            '''Return posterior for ellipse shape for a given theta'''
            a = theta[3]
            b = theta[4]
            #Ellipse_vector = _Init_Ellipse(theta)
            Ellipse_vector = _Distances_from_center_of_ellipse(theta,self.Z)
            sigma = 0.09
            sigma_prior = np.array(rospy.get_param('/object_list/o'+str(self.class_number) + '/cov'))
            L = GMM_Prior(1,sigma,Ellipse_vector)
            a_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/a')
            b_mean = rospy.get_param('/object_list/o'+str(self.class_number) + '/b')
            
            Prior = GMM_Prior(np.array([a,b]),sigma_prior,np.array([a_mean,b_mean]))

            return -Prior * L * self.Probability[self.class_number-1]

            





        
