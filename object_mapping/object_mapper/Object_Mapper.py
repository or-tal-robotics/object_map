#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from obj_class import Updated_Probabilities_and_Cls,closest_node,Object_Map_cls,Search_Radius
import pandas as pd

# Msgs:
from joint_object_localizer.msg import OG_List
from object_mapping.msg import Object,Object_Map


def OG_callback(data):

    # The subs:
    global Theta_list
    Theta_list = data
    

# Initial node:
rospy.init_node('List_of_mapped_objects', anonymous=True)
# Publishers:
OM_publisher = rospy.Publisher('/object_mapped_values',Object_Map,queue_size=10)
# Subscribers:
Theta_list_sub = rospy.Subscriber('/Theta_List',OG_List,OG_callback)
rospy.wait_for_message('/Theta_List',OG_List)

object_mapped_values = Object_Map()
object_class_list = []
objects_center = np.array([0,0])

r = rospy.Rate(5)

while not rospy.is_shutdown():
    r.sleep()
    # Globals:
    global Theta_list
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
        new_center = np.array([[data.object_list[ii].x_center,data.object_list[ii].y_center]])
        
        # The first object initializer:
        if len(object_class_list) == 0:
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

            continue

        
        dist,index = closest_node(new_center,objects_center)
        env = Search_Radius(3*object_class_list[index].r,2*object_class_list[index].a,2*object_class_list[index].b)

        if dist > env:
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
        
        else:
            object_class_list[index].prob_distribution,object_class_list[index].cls_num = Updated_Probabilities_and_Cls(object_class_list[index].prob_distribution,obj_new.probabilities,obj_new.cls_num)
            object_mapped_values.object_map[index].probabilities = object_class_list[index].prob_distribution
            object_mapped_values.object_map[index].cls_num = object_class_list[index].cls_num

        OM_publisher.publish(object_mapped_values)