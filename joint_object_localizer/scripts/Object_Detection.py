#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from scipy.optimize import differential_evolution
from sklearn.neighbors import NearestNeighbors
from tf.transformations import quaternion_from_euler
from geometric_functions import _Init_Ellipse,_Initializing_half_Rectangle,quaternion_to_euler,New_Ce_array,_distance,closest_node

# Msgs:
from object_detector_ssd_tf_ros.msg import SSD_Outputs
from joint_object_localizer.msg import Object_Geometry,OG_List
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# test:
#from visualization_msgs.msg import Marker

# Callback functions:
def callback_laser(data):
    global scan_point
    scan_point = data

def Robot_callback(data):
    global Robot_Pose
    Robot_Pose = data

def SSD_callback(data):
        
    global SSD_info
    SSD_info = data
        



# ----------------------Finding the center with DE algoritm------------------------:
# lieklihood functions:
def probability_for_can(theta):
    size_of_r_new = len(circle[:,0]) - 1
    r_new = theta[2] * np.ones((size_of_r_new,1))
    r_c = ( (theta[0]-circle[:,0])**2 + (theta[1]-circle[:,1])**2 )**0.5
    return -np.sum(np.exp(-20*(r_new-r_c)**2))* np.exp(-0.7*(theta[2]-0.06)**2)

def Ellipse_likelihood(theta,nbrs_obj):

        a = theta[3]
        b = theta[4]
        Ellipse_vector = _Init_Ellipse(theta)
        distances, idc = nbrs_obj.kneighbors(Ellipse_vector.T)
        
        return -np.sum(np.exp(-10*distances**2) * np.exp(-0.4*(a - 0.25)**2) * np.exp(-0.4*(b - 0.1)**2))

def Rectangle_Likelihood(theta, nbrs_obj):

        a = theta[3]
        b = theta[4]
        Rectangle_vector = _Initializing_half_Rectangle(theta).T       
        distances, idc = nbrs_obj.kneighbors(Rectangle_vector.T)

        return -np.sum(np.exp(-10*distances**2) * np.exp(-0.2*(a - 0.5)**2) * np.exp(-0.2*(b - 0.5)**2))


rospy.init_node('Theta_values', anonymous=True)
# Publisher:
Theta_Publisher = rospy.Publisher('/Theta_Object',Object_Geometry,queue_size = 5 )
Theta_list_pub = rospy.Publisher('/Theta_List',OG_List,queue_size = 10)
# Subscribers:
scan_point_sub = rospy.Subscriber('/scan' ,LaserScan , callback_laser )
rospy.wait_for_message('/scan',LaserScan)
Robot_sub = rospy.Subscriber('/slam_out_pose',PoseStamped,Robot_callback)
rospy.wait_for_message('/slam_out_pose',PoseStamped)
SSD_sub = rospy.Subscriber('/im_info',SSD_Outputs,SSD_callback,queue_size=1)
rospy.wait_for_message('/im_info',SSD_Outputs)

r = rospy.Rate(5)

while not rospy.is_shutdown():
    r.sleep()
    data = SSD_info
    if data.outputs[0].cls == -1:
        Theta_list = OG_List()
        Theta_list.object_list = []
        Theta_list_pub.publish(Theta_list)
        continue
    Theta_list = OG_List()
    # globals
    Theta_list.object_list = []
    # Calculating the maximum angle of the sensor:
    angle_jumps = scan_point.angle_increment
    phi_max = len(scan_point.ranges) * angle_jumps
    theta_camera_max = 1.4

    # The angle that needed to be added to the camera angle to get the angle for the sensor:
    correction_for_sensor = (phi_max + theta_camera_max) / 2
    size =  (len(scan_point.ranges) - 1)/2
    
    
    # In case there is no object that has been detected:
    
    
    for ii in range (0,len(data.outputs)):
        Theta_Object = Object_Geometry()
        # Getting the info from the camera:
        x_min_new = data.outputs[ii].x_min 
        x_max_new = data.outputs[ii].x_max
        cls_num = data.outputs[ii].cls

        
        # Theta from camera:
        theta_min = x_min_new*(1.4/300)
        theta_max = x_max_new*(1.4/300)
        # Angle from the sensor:
        angle_max = int((correction_for_sensor - theta_min) / angle_jumps)
        angle_min = int((correction_for_sensor - theta_max) / angle_jumps)

        # Orientation of the robot:
        R_qx = Robot_Pose.pose.orientation.x
        R_qy = Robot_Pose.pose.orientation.y
        R_qz = Robot_Pose.pose.orientation.z
        R_qw = Robot_Pose.pose.orientation.w
        [yaw, pitch, roll] = quaternion_to_euler(R_qx,R_qy,R_qz,R_qw)
        
        # Location of the robot: 
        x_R = Robot_Pose.pose.position.x
        y_R = Robot_Pose.pose.position.y

        las_points = np.zeros((2*size + 1,2))
        # Initializing the array of the points:
        circle = []

        R = np.array(scan_point.ranges)
        # Initializing an inf values:
        R[np.isinf(R)] = 0

        for i in range (0,size):
            
            if R[i] == 0:
                las_points[i,0] = 100000
                las_points[i,1] = 100000
                continue
            las_points[i,0] = -R[i] * np.cos((size - i) * angle_jumps - np.pi)
            las_points[i,1] = R[i] * np.sin((size - i) * angle_jumps - np.pi)
            
        for i in range (size,2*size+1):
            if R[i] == 0:
                las_points[i,0] = 100000
                las_points[i,1] = 100000
                continue
            las_points[i,0] = R[i] * np.cos((i-size) * angle_jumps)
            las_points[i,1] = R[i] * np.sin((i-size) * angle_jumps)
        
        # Round objects:
        if cls_num == 5:
            
            
            # Initializing the circle array:
            circle = np.array((las_points[angle_min:angle_max,:]))

            # Bounds for DE algoritm
            bounds = [ [np.amin(circle[:,0])-0.1 , np.amin([np.amax(circle[:,0])+0.1 , 2.5])] , \
                [np.amin(circle[:,1])-0.1,np.amin([np.amax(circle[:,1])+0.1 , 2.5])],[0.02,0.08]]
            
            # DE algoritm
            circle_minimized_values = differential_evolution(probability_for_can,bounds \
                , maxiter = 100,  popsize=15, tol=0.00001)

            # Entering the found data:
            v0 = New_Ce_array(circle_minimized_values.x[0],circle_minimized_values.x[1],x_R,y_R,yaw)
            Theta_Object.probabilities = data.outputs[ii].probability_distribution
            Theta_Object.x_center = v0[0]
            Theta_Object.y_center = v0[1]
            Theta_Object.r = circle_minimized_values.x[2]
            Theta_Object.a = 0
            Theta_Object.b = 0
            Theta_Object.angle = 0

            Theta_Object.cls = cls_num
            
            Theta_list.object_list.append(Theta_Object)
            #Theta_Publisher.publish(Theta_Object)
            
        # Box:
        elif cls_num == 18 or cls_num == 20:

            
            # Initializing the box array:
            box = np.array((las_points[angle_min:angle_max,:]))
            # Bounds for DE algoritm
            bounds_R = [ [np.amin(box[:,0])-0.1 , np.amin([np.amax(box[:,0])+0.1 , 3.5])] , \
                [np.amin(box[:,1])-0.1,np.amin([np.amax(box[:,1])+0.1 , 3.5])],[-np.pi,np.pi],[0.2,0.8],[0.2,0.8]]

            nbrs_obj = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(box)
            # For being able to send two values for DE algoritm:
            iterable_function = lambda x: Rectangle_Likelihood(x,nbrs_obj)
            # DE angoritm:
            rectangle_minimized_values = differential_evolution(iterable_function,bounds_R \
                , maxiter = 100,  popsize=15, tol=0.00001)
            # Entering the found data:
            Theta_Object.probabilities = data.outputs[ii].probability_distribution
            Theta_Object.x_center , Theta_Object.y_center = New_Ce_array(rectangle_minimized_values.x[0] ,rectangle_minimized_values.x[1],x_R,y_R,yaw)
            Theta_Object.a = rectangle_minimized_values.x[3]
            Theta_Object.b = rectangle_minimized_values.x[4]
            Theta_Object.angle = rectangle_minimized_values.x[2] + yaw
            Theta_Object.r = 0
            Theta_Object.cls = cls_num
            
            Theta_list.object_list.append(Theta_Object)
            #Theta_Publisher.publish(Theta_Object)
            
        # cat,dog,horse:
        elif cls_num == 8 or cls_num == 12 or cls_num == 13:
            
            # Initializing the box array:
            ellipse = np.array((las_points[angle_min:angle_max,:]))
            # Bounds for DE algoritm
            bounds_E = [ [np.amin(ellipse[:,0])-0.1 , np.amin([np.amax(ellipse[:,0])+0.1 , 3.5])] , \
                [np.amin(ellipse[:,1])-0.1,np.amin([np.amax(ellipse[:,1])+0.1 , 3.5])],[-np.pi,np.pi],[0.1,0.8],[0.1,0.8]]

            nbrs_obj = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(ellipse)
            # For being able to send two values for DE algoritm:
            iterable_function = lambda x: Ellipse_likelihood(x,nbrs_obj)
            # DE angoritm:
            elliptical_minimized_values = differential_evolution(iterable_function,bounds_E \
                , maxiter = 100,  popsize=15, tol=0.00001)
            # Entering the found data:
            Theta_Object.probabilities = data.outputs[ii].probability_distribution
            Theta_Object.x_center , Theta_Object.y_center = New_Ce_array(elliptical_minimized_values.x[0] ,elliptical_minimized_values.x[1],x_R,y_R,yaw)
            Theta_Object.a = elliptical_minimized_values.x[3]
            Theta_Object.b = elliptical_minimized_values.x[4]
            Theta_Object.angle = elliptical_minimized_values.x[2] + yaw
            Theta_Object.r = 0
            Theta_Object.cls = cls_num
            #Theta_Publisher.publish(Theta_Object)
            Theta_list.object_list.append(Theta_Object)
            
            '''
            # -----------------------Printing test----------------:
            cat = Marker()
            # Test for putting a can in the map.
            cat.header.frame_id = 'map'
            
            # Colore of can
            cat.color.r = 100
            cat.color.g = 0
            cat.color.b = 20
            cat.color.a = 1
            # Rest of things:
            cat.id = 1
            cat.type = 3
            cat.action = 0
            cat.lifetime.secs = 0

            # Update the new center:
            cat.pose.position.x = Theta_Object.x_center
            cat.pose.position.y = Theta_Object.y_center
            cat.pose.position.z = 0.15
            # Size of the can
            cat.scale.x = 2 * Theta_Object.a
            cat.scale.y = 2 * Theta_Object.b
            cat.scale.z = 0.3

            [x_b,y_b,z_b,w_b] = quaternion_from_euler(0,0,Theta_Object.angle)
            # Orientation of the can
            cat.pose.orientation.x = x_b
            cat.pose.orientation.y = y_b
            cat.pose.orientation.z = z_b
            cat.pose.orientation.w = w_b

            cat.ns = 'cat'
            cat.header.stamp = rospy.Time.now()

            cat_pub.publish(cat)
            '''
            
    Theta_list_pub.publish(Theta_list)


# Test:
##cat_pub = rospy.Publisher('/Cat' , Marker , queue_size = 5)






    