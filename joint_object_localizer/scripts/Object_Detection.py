#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from scipy.optimize import differential_evolution
from tf.transformations import quaternion_from_euler
from geometric_functions import Likelihood, quaternion_to_euler, New_Ce_array

# Msgs:
from object_detector_ssd_tf_ros.msg import SSD_Outputs
from joint_object_localizer.msg import Object_Geometry,OG_List
from nav_msgs.msg import Odometry
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

def Odom_callback(data):
    global Robot_Odometry
    Robot_Odometry = data.pose.pose
        

rospy.init_node('Theta_values', anonymous=True)
# Publisher:

Theta_list_pub = rospy.Publisher('/Theta_List',OG_List,queue_size = 10)
# Subscribers:
scan_point_sub = rospy.Subscriber('/scan' ,LaserScan , callback_laser )
rospy.wait_for_message('/scan',LaserScan)
Robot_sub = rospy.Subscriber('/slam_out_pose',PoseStamped,Robot_callback)
rospy.wait_for_message('/slam_out_pose',PoseStamped)
Robot_World_Location_sub = rospy.Subscriber('/odom',Odometry,Odom_callback)
rospy.wait_for_message('/odom',Odometry)
SSD_sub = rospy.Subscriber('/im_info',SSD_Outputs,SSD_callback,queue_size=1)
rospy.wait_for_message('/im_info',SSD_Outputs)

# YAMLs:
A_Round = rospy.get_param('/Array/round')
A_Rectangle = rospy.get_param('/Array/rectangle')
A_Elliptical = rospy.get_param('/Array/elliptical')
Object_cls_list = np.array(rospy.get_param('/Array/object_list'))
correction_factor = rospy.get_param('/Cord_cor/Simulation/Correction')

r = rospy.Rate(5)
global Robot_Odometry

while not rospy.is_shutdown():
    r.sleep()

    data = SSD_info
    if data.outputs[0].cls == -1:
        Theta_list = OG_List()
        Theta_list.object_list = []
        Theta_list_pub.publish(Theta_list)
        continue
    Theta_list = OG_List()
    Theta_list.object_list = []
    # Calculating the maximum angle of the sensor:
    angle_jumps = scan_point.angle_increment
    phi_max = len(scan_point.ranges) * angle_jumps
    theta_camera_max = 1.4

    # The angle that needed to be added to the camera angle to get the angle for the sensor:
    correction_for_sensor = (phi_max + theta_camera_max) / 2
    size =  (len(scan_point.ranges) - 1)/2
    
    
    
    for ii in range (0,len(data.outputs)):

        Theta_Object = Object_Geometry()
        # Getting the info from the camera:
        x_min_new = data.outputs[ii].x_min 
        x_max_new = data.outputs[ii].x_max
        y_min_new = data.outputs[ii].y_min
        y_max_new = data.outputs[ii].y_max

        if y_min_new > 160 or y_max_new < 130:
            continue


        cls_num = data.outputs[ii].cls
        Theta_Object.height_factor = data.outputs[ii].height_factor
        
        # In case the object is not a one that can be stationed in the map:
        if cls_num not in Object_cls_list:
            continue
        

        # Theta from camera:
        theta_min = x_min_new*(1.4/300)
        theta_max = x_max_new*(1.4/300)

        # Angle from the sensor:
        angle_max = int((correction_for_sensor - theta_min) / angle_jumps)
        angle_min = int((correction_for_sensor - theta_max) / angle_jumps)

        
        # Orientation of the robot from slam_out_pose:
        R_qx = Robot_Pose.pose.orientation.x
        R_qy = Robot_Pose.pose.orientation.y
        R_qz = Robot_Pose.pose.orientation.z
        R_qw = Robot_Pose.pose.orientation.w
        '''
        # Orientation of the robot from the Odometry:
        R_qx = Robot_Odometry.orientation.x
        R_qy = Robot_Odometry.orientation.y
        R_qz = Robot_Odometry.orientation.z
        R_qw = Robot_Odometry.orientation.w
        '''
        [yaw, pitch, roll] = quaternion_to_euler(R_qx,R_qy,R_qz,R_qw)
        
        
        # Location of the robot from slam_out_pose: 
        x_R = Robot_Pose.pose.position.x
        y_R = Robot_Pose.pose.position.y
        '''
        # Location of the robot from Odometry: 
        x_R = Robot_Odometry.position.x
        y_R = Robot_Odometry.position.y
        '''
        las_points = np.zeros((2*size + 1,2))
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
        if cls_num in A_Round:
            
            
            # Initializing the circle array:
            circle = np.array((las_points[angle_min:angle_max,:]))

            if np.amin(circle[:,0]**2 + circle[:,1]**2)>4:
                continue

            bound_r = rospy.get_param('/object_list/o'+str(cls_num)+'/bound_r')

            # Bounds for DE algoritm
            bounds = [ [np.amin(circle[:,0])-0.1 , np.amin([np.amax(circle[:,0])+0.1 , 2.5])] , \
                [np.amin(circle[:,1])-0.1,np.amin([np.amax(circle[:,1])+0.1 , 2.5])],bound_r,[-np.pi,np.pi]]
            
            
            C_class = Likelihood(class_number=cls_num,Z=circle,SSD_probability=data.outputs[ii].probability_distribution)
            # DE algoritm
            circle_minimized_values = differential_evolution(C_class.probability_for_circle,bounds \
                , maxiter = 100,  popsize=15, tol=0.00001)

            if circle_minimized_values.x[0]**2 + circle_minimized_values.x[1]**2 > 4:
                continue
            # Entering the found data:
            v0 = New_Ce_array(circle_minimized_values.x[0],circle_minimized_values.x[1],x_R,y_R,yaw,correction_factor)
            Theta_Object.probabilities = data.outputs[ii].probability_distribution
            Theta_Object.x_center = v0[0]
            Theta_Object.y_center = v0[1]
            Theta_Object.r = circle_minimized_values.x[2]
            Theta_Object.a = 0
            Theta_Object.b = 0
            Theta_Object.angle = 0

            Theta_Object.cls = cls_num
            
            Theta_list.object_list.append(Theta_Object)
            
            
        # Box:
        elif cls_num in A_Rectangle:

            
            # Initializing the box array:
            box = np.array((las_points[angle_min:angle_max,:]))

            if np.amin(box[:,0]**2 + box[:,1]**2)>4:
                continue

            bound_a = rospy.get_param('/object_list/o'+str(cls_num)+'/bound_a')
            bound_b = rospy.get_param('/object_list/o'+str(cls_num)+'/bound_b')
            # Bounds for DE algoritm
            bounds_R = [ [np.amin(box[:,0])-0.1 , np.amin([np.amax(box[:,0])+0.1 , 3])] , \
                [np.amin(box[:,1])-0.1,np.amin([np.amax(box[:,1])+0.1 , 3])],[-np.pi,np.pi],bound_a,bound_b]

            
            R_class = Likelihood(class_number=cls_num , Z=box , SSD_probability=data.outputs[ii].probability_distribution)
            # DE angoritm:
            rectangle_minimized_values = differential_evolution(R_class.probability_for_Rectangle,bounds_R \
                , maxiter = 100,  popsize=15, tol=0.00001)
            # Entering the found data:


            if rectangle_minimized_values.x[0]**2 + rectangle_minimized_values.x[1]**2 > 9:
                continue
            Theta_Object.probabilities = data.outputs[ii].probability_distribution
            Theta_Object.x_center , Theta_Object.y_center = New_Ce_array(rectangle_minimized_values.x[0] ,rectangle_minimized_values.x[1],x_R,y_R,yaw,correction_factor)
            Theta_Object.a = rectangle_minimized_values.x[3]
            Theta_Object.b = rectangle_minimized_values.x[4]
            Theta_Object.angle = rectangle_minimized_values.x[2] + yaw
            Theta_Object.r = 0
            Theta_Object.cls = cls_num
            
            
            Theta_list.object_list.append(Theta_Object)
            
        # cat,dog:
        elif cls_num in A_Elliptical:
            
            # Initializing the box array:
            ellipse = np.array((las_points[angle_min:angle_max,:]))

            if np.amin(ellipse[:,0]**2 + ellipse[:,1]**2)>4:
                continue

            bound_a = rospy.get_param('/object_list/o'+str(cls_num)+'/bound_a')
            bound_b = rospy.get_param('/object_list/o'+str(cls_num)+'/bound_b')
            # Bounds for DE algoritm
            bounds_E = [ [np.amin(ellipse[:,0])-0.1 , np.amin([np.amax(ellipse[:,0])+0.1 , 2])] , \
                [np.amin(ellipse[:,1])-0.1,np.amin([np.amax(ellipse[:,1])+0.1 , 2])],[0,np.pi],bound_a,bound_b]

            E_class = Likelihood(class_number=cls_num , Z=ellipse , SSD_probability=data.outputs[ii].probability_distribution)
            # DE angoritm:
            elliptical_minimized_values = differential_evolution(E_class.probability_for_Ellipse,bounds_E \
                , maxiter = 100,  popsize=15, tol=0.00001)

                  
            if elliptical_minimized_values.x[0]**2 + elliptical_minimized_values.x[1]**2 > 4:
                continue
            # Entering the found data:
            Theta_Object.probabilities = data.outputs[ii].probability_distribution
            Theta_Object.x_center , Theta_Object.y_center = New_Ce_array(elliptical_minimized_values.x[0] ,elliptical_minimized_values.x[1],x_R,y_R,yaw,correction_factor)
            Theta_Object.a = elliptical_minimized_values.x[3]
            Theta_Object.b = elliptical_minimized_values.x[4]
            Theta_Object.angle = elliptical_minimized_values.x[2] + yaw
            Theta_Object.r = 0
            Theta_Object.cls = cls_num
            #Theta_Publisher.publish(Theta_Object)
            Theta_list.object_list.append(Theta_Object)
       
            
            
    Theta_list_pub.publish(Theta_list)








    