#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from obj_class import Updated_Probabilities_and_Cls,closest_node,Object_Map_cls,Search_Radius,Theta_updater
import pandas as pd

# Msgs:
from joint_object_localizer.msg import OG_List
from object_mapping.msg import Object,Object_Map

global Theta_list

def OG_callback(data):

    # The subs:
    global Theta_list
    Theta_list = data

    
# Initial node:
rospy.init_node('List_of_mapped_objects', anonymous=True)
# Publishers:
OM_publisher = rospy.Publisher('/object_mapped_values',Object_Map,queue_size=10)
A_Rectangle = rospy.get_param('/Array/rectangle')
A_Elliptical = rospy.get_param('/Array/elliptical')

rr = rospy.Rate(10)
object_class_list = []
object_mapped_values = Object_Map()    
objects_center = np.array([0,0])

while not rospy.is_shutdown():
    
    # Subscribers:
    Theta_list_sub = rospy.Subscriber('/Theta_List',OG_List,OG_callback,queue_size=7)
    rospy.wait_for_message('/Theta_List',OG_List)

    data = Theta_list
    
    for ii in range(0,len(data.object_list)):
        
        
        # Getting the new data:
        obj_new = Object()
        obj_new.x_center = data.object_list[ii].x_center
        obj_new.y_center = data.object_list[ii].y_center
        obj_new.r = data.object_list[ii].r
        obj_new.a = data.object_list[ii].a
        obj_new.b = data.object_list[ii].b
        obj_new.angle = data.object_list[ii].angle
        obj_new.probabilities = data.object_list[ii].probabilities
        obj_new.cls_num = data.object_list[ii].cls
        obj_new.height_factor = data.object_list[ii].height_factor
        obj_new.object_height = data.object_list[ii].object_height
        new_center = np.array([[data.object_list[ii].x_center,data.object_list[ii].y_center]])
        
        
        # The first object initializer:
        if len(object_class_list) == 0:
            print ('Adding first object.')
            # Adding the first object to the map msg:
            object_mapped_values.object_map.append(obj_new)
            OM_publisher.publish(object_mapped_values)
            # Adding the first object to the map class:
            object_class = Object_Map_cls()
            object_class.x_center = obj_new.x_center
            object_class.y_center = obj_new.y_center
            object_class.r = obj_new.r
            object_class.a = obj_new.a
            object_class.b = obj_new.b
            object_class.angle = obj_new.angle
            object_class.probabilities = obj_new.probabilities
            object_class.cls_num = obj_new.cls_num
            object_class_list.append(object_class)
            # Adding the first center to numpy array:
            objects_center = np.array([[object_class.x_center,object_class.y_center]])
            print ('Added first object.')
            continue

        
        dist,index = closest_node(new_center,objects_center)

        env = Search_Radius(3*object_class_list[index].r,object_class_list[index].a,object_class_list[index].b)
        
        if index in A_Elliptical:
            env = env * 4
        elif index in A_Rectangle:
            env = env * 1.5

        if dist > env:
            print ('Adding a new object.')
            # Adding the object to the map msg:
            object_mapped_values.object_map.append(obj_new)
            OM_publisher.publish(object_mapped_values)
            # Adding the object to the map class:
            object_class = Object_Map_cls()
            object_class.x_center = obj_new.x_center
            object_class.y_center = obj_new.y_center
            object_class.r = obj_new.r
            object_class.a = obj_new.a
            object_class.b = obj_new.b
            object_class.angle = obj_new.angle
            object_class.probabilities = obj_new.probabilities
            object_class.cls_num = obj_new.cls_num
            object_class_list.append(object_class)
            # Adding the center to numpy array:
            objects_center = np.concatenate((objects_center,new_center))
            print ('Added a new object.')
        
        else:
            print ('Start to update.')
            object_class_list[index].prob_distribution ,object_class_list[index].cls_num = Updated_Probabilities_and_Cls(object_class_list[index].prob_distribution,obj_new.probabilities,obj_new.cls_num)

            object_mapped_values.object_map[index].probabilities = object_class_list[index].prob_distribution

            object_mapped_values.object_map[index].cls_num = np.int16(object_class_list[index].cls_num)

            # Updating theta:
            [x,y,r,a,b,phi] = Theta_updater(obj_new.x_center,object_class_list[index].x_center,
                                             obj_new.y_center,object_class_list[index].y_center,
                                             obj_new.r,object_class_list[index].r,
                                             obj_new.a,object_class_list[index].a,
                                             obj_new.b,object_class_list[index].b,
                                             obj_new.angle,object_class_list[index].angle)
            
            object_class_list[index].x_center = x
            object_class_list[index].y_center = y
            object_class_list[index].r = r
            object_class_list[index].a = a
            object_class_list[index].b = b
            object_class_list[index].angle = phi

            object_mapped_values.object_map[index].x_center = x
            object_mapped_values.object_map[index].y_center = y
            object_mapped_values.object_map[index].r = r
            object_mapped_values.object_map[index].a = a
            object_mapped_values.object_map[index].b = b
            object_mapped_values.object_map[index].angle = phi


            print ('Updated an existed object.')
        
        OM_publisher.publish(object_mapped_values)
    #rr.sleep()
    