#!/usr/bin/env python

# Libraries:
import rospy
from tf.transformations import quaternion_from_euler
import numpy as np

# msgs:
from object_mapping.msg import M
from visualization_msgs.msg import Marker , MarkerArray

def callback_map(data):
    global O_map
    O_map = data


rospy.init_node('Rviz_Objects_node', anonymous=True)

# Publisher:
O_publisher = rospy.Publisher('/Rviz_Objects' , MarkerArray , queue_size = 10)

#Subsctibers:
M_Subs = rospy.Subscriber('/M',M,callback_map)
rospy.wait_for_message('/M',M)

global O_map

A_Round = rospy.get_param('/Array/round')
A_Rectangle = rospy.get_param('/Array/rectangle')
A_Elliptical = rospy.get_param('/Array/elliptical')

slp = rospy.Rate(5)

while not rospy.is_shutdown():
    
    M_data = O_map
    slp.sleep()
    list_marker = MarkerArray()
    list_marker.markers = []
    

    for ii in range(len(M_data.M)):
        marker = Marker()
        marker_name = Marker()
        probability = []
        for rr in range (0,len(M_data.M[ii].m_i)):
            probability.append(M_data.M[ii].m_i[rr].probability)
        max_p = np.argmax(np.array(probability))
        # For the angle:
        [x_q,y_q,z_q,w_q] = quaternion_from_euler(0,0,M_data.M[ii].m_i[max_p].angle)

        name = rospy.get_param('/object_list/o'+str(M_data.M[ii].m_i[max_p].cls_num)+'/name')
        
        height_factor = M_data.M[ii].m_i[max_p].height_factor
        if height_factor == 0:
            height_factor = 1
        
        if M_data.M[ii].m_i[max_p].cls_num in A_Round:

            # Test for putting a can in the map.
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            # Colore of can
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 200
            marker.color.a = 1
            # Rest of things:
            marker.id = 1
            marker.type = 3
            marker.action = 0
            marker.lifetime.secs = 0
            # Orientation of the can
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            # Size:        
            marker.scale.x = 2 * M_data.M[ii].m_i[max_p].r
            marker.scale.y = 2 * M_data.M[ii].m_i[max_p].r
            marker.scale.z = height_factor * marker.scale.x
            # Location of the can:
            marker.pose.position.x = M_data.M[ii].m_i[max_p].x_center
            marker.pose.position.y = M_data.M[ii].m_i[max_p].y_center
            marker.pose.position.z =  marker.scale.z/2
            # Name:
            marker.ns = name + str(ii)
            list_marker.markers.append(marker)

            # Text adding:
            marker_name.header.frame_id = 'map'
            marker_name.header.stamp = rospy.Time.now()
            # Orientation of the text
            marker_name.pose.orientation.x = 0
            marker_name.pose.orientation.y = 0
            marker_name.pose.orientation.z = 0
            marker_name.pose.orientation.w = 1
            # Colore of text
            marker_name.color.r = 200
            marker_name.color.g = 200
            marker_name.color.b = 200
            marker_name.color.a = 1
            # Rest of things:
            marker_name.id = 10
            marker_name.type = 9
            marker_name.action = 0
            marker_name.lifetime.secs = 0
            marker_name.pose.position.x = M_data.M[ii].m_i[max_p].x_center
            marker_name.pose.position.y = M_data.M[ii].m_i[max_p].y_center
            marker_name.pose.position.z = 2*marker.pose.position.z + 0.1
            # Size of the text
            marker_name.scale.x = 0.2
            marker_name.scale.y = 0.2
            marker_name.scale.z = 0.2
            marker_name.text = name
            marker_name.ns = name + str(ii)

            list_marker.markers.append(marker_name)
            continue


        
        if M_data.M[ii].m_i[max_p].cls_num in A_Rectangle:
            # Test for putting a TV in the map.
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            # Colour of TV
            marker.color.r = 100
            marker.color.g = 0
            marker.color.b = 100
            marker.color.a = 1
            # Rest of things:
            marker.id = 1
            marker.type = 1
            marker.action = 0
            marker.lifetime.secs = 0
            # Orientation of the TV
            marker.pose.orientation.x = x_q
            marker.pose.orientation.y = y_q
            marker.pose.orientation.z = z_q
            marker.pose.orientation.w = w_q
            # Size:        
            marker.scale.x = M_data.M[ii].m_i[max_p].a
            marker.scale.y = M_data.M[ii].m_i[max_p].b
            marker.scale.z = height_factor*(np.absolute(marker.scale.x*np.cos(M_data.M[ii].m_i[max_p].angle)) + np.absolute(marker.scale.y*np.sin(M_data.M[ii].m_i[max_p].angle)))
            # Location of the TV:
            marker.pose.position.x = M_data.M[ii].m_i[max_p].x_center
            marker.pose.position.y = M_data.M[ii].m_i[max_p].y_center
            marker.pose.position.z = marker.scale.z/2
            # Name:
            marker.ns = name + str(ii)
            list_marker.markers.append(marker)

            # Text adding:
            marker_name.header.frame_id = 'map'
            marker_name.header.stamp = rospy.Time.now()
            # Orientation of the text
            marker_name.pose.orientation.x = x_q
            marker_name.pose.orientation.y = y_q
            marker_name.pose.orientation.z = z_q
            marker_name.pose.orientation.w = w_q
            # Colour of text
            marker_name.color.r = 200
            marker_name.color.g = 200
            marker_name.color.b = 200
            marker_name.color.a = 1
            # Rest of things:
            marker_name.id = 10
            marker_name.type = 9
            marker_name.action = 0
            marker_name.lifetime.secs = 0
            marker_name.pose.position.x = M_data.M[ii].m_i[max_p].x_center
            marker_name.pose.position.y = M_data.M[ii].m_i[max_p].y_center
            marker_name.pose.position.z = 2*marker.pose.position.z + 0.1
            # Size of the text
            marker_name.scale.x = 0.2
            marker_name.scale.y = 0.2
            marker_name.scale.z = 0.2
            marker_name.text = name
            marker_name.ns = name + str(ii)

            list_marker.markers.append(marker_name)
            continue

        if M_data.M[ii].m_i[max_p].cls_num in A_Elliptical:
            # Test for putting an elliptical in the map.
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            
            # Rest of things:
            marker.id = 1
            marker.type = 3
            marker.action = 0
            marker.lifetime.secs = 0
            # Orientation of the TV
            marker.pose.orientation.x = x_q
            marker.pose.orientation.y = y_q
            marker.pose.orientation.z = z_q
            marker.pose.orientation.w = w_q
            # Size:        
            marker.scale.x = 2*M_data.M[ii].m_i[max_p].a
            marker.scale.y = 2*M_data.M[ii].m_i[max_p].b
            marker.scale.z = 2*(np.absolute(marker.scale.x*np.cos(M_data.M[ii].m_i[max_p].angle)) + np.absolute(marker.scale.y*np.sin(M_data.M[ii].m_i[max_p].angle)))*height_factor
            # Location of the TV:
            marker.pose.position.x = M_data.M[ii].m_i[max_p].x_center
            marker.pose.position.y = M_data.M[ii].m_i[max_p].y_center
            marker.pose.position.z = marker.scale.z/2
            
            
            marker.color.r = 50
            marker.color.g = 50
            marker.color.b = 100
            marker.color.a = 1
            marker.ns = name + str(ii)
            marker_name.text = name
            marker_name.ns = name + str(ii)
            
            
            list_marker.markers.append(marker)

            # Text adding:
            marker_name.header.frame_id = 'map'
            marker_name.header.stamp = rospy.Time.now()
            # Orientation of the text
            marker_name.pose.orientation.x = x_q
            marker_name.pose.orientation.y = y_q
            marker_name.pose.orientation.z = z_q
            marker_name.pose.orientation.w = w_q
            # Colour of text
            marker_name.color.r = 200
            marker_name.color.g = 200
            marker_name.color.b = 200
            marker_name.color.a = 1
            # Rest of things:
            marker_name.id = 10
            marker_name.type = 9
            marker_name.action = 0
            marker_name.lifetime.secs = 0
            marker_name.pose.position.x = M_data.M[ii].m_i[max_p].x_center
            marker_name.pose.position.y = M_data.M[ii].m_i[max_p].y_center
            marker_name.pose.position.z = 2*marker.pose.position.z + 0.1
            # Size of the text
            marker_name.scale.x = 0.2
            marker_name.scale.y = 0.2
            marker_name.scale.z = 0.2

            list_marker.markers.append(marker_name)
            continue

    #publish:
    O_publisher.publish(list_marker)
    for jj in range(len(list_marker.markers)):
        list_marker.markers[jj].action = 2
            

