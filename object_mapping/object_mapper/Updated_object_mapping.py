#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
from H_functions import SO_class, q_i , making_M , Prob_updater
import pandas as pd

# Msgs:
from joint_object_localizer.msg import M_Suggested_List
from object_mapping.msg import Single_Class , M_i , M

global last_location_x , last_location_y
last_location_x = 0
last_location_y = 0

def M_o_callback(data):

    global epsilon
    global M_list_class
    global last_location_x , last_location_y
    # The subs:
    global Theta_list
    Theta_list = data
    
    x = []
    y = []
    r = []
    a = []
    b = []
    angle = []
    cls_num = []
    Prb = []
    
    # Updating the msg:   
    for jj in range(0,len(data.M_list)):
        
        # Arranging the data for Mo:
        for i in range(0,len(data.M_list[jj].object_list)):

            # getting the fresh data
            x.append(data.M_list[jj].object_list[i].x_center)
            y.append(data.M_list[jj].object_list[i].y_center)
            r.append(data.M_list[jj].object_list[i].r)
            a.append(data.M_list[jj].object_list[i].a)
            b.append(data.M_list[jj].object_list[i].b)
            angle.append(data.M_list[jj].object_list[i].angle)
            cls_num.append(data.M_list[jj].object_list[i].cls)
            Prb.append(data.M_list[jj].object_list[i].Final_Likelihood)

        # In case the same object has been sent more than once:
        if x[0] == last_location_x  and y[0] == last_location_y:
            return 0

        last_location_x = x[0]
        last_location_y = y[0]

        # The new object:
        Mo = SO_class(x_center=x,y_center=y,r=r,a=a,b=b,angle=angle,
                        cls_num=cls_num,prob_distribution=Prb)
        
        print 'Got new Mo'
        if len(M_list_class) == 0:
            print "First Object"
            M_list_class.append(Mo)
            
            
        else:
            q = []
            for ii in range(0,len(M_list_class)):
                q.append(q_i(M_list_class[ii],Mo))
                
            ii = np.argmax(q)

            # New object
            if q[ii] < epsilon:
                print 'Added an object'
                M_list_class.append(Mo)
                
            # Updating an object:
            else:
                print 'Updating an object'
                alpha = 0.5

                # Updating:
                M_list_class[ii].prob_distribution = Prob_updater(alpha,
                                                                  M_list_class[ii].prob_distribution,
                                                                  Mo.prob_distribution)

                M_list_class[ii].x_center = alpha * M_list_class[ii].x_center\
                                                    + (1-alpha)*Mo.x_center
                M_list_class[ii].y_center = alpha * M_list_class[ii].y_center\
                                                    + (1-alpha)*Mo.y_center 
                M_list_class[ii].r = alpha * M_list_class[ii].r + (1-alpha)*Mo.r
                M_list_class[ii].a = alpha * M_list_class[ii].a + (1-alpha)*Mo.a
                M_list_class[ii].b = alpha * M_list_class[ii].b + (1-alpha)*Mo.b
                M_list_class[ii].angle = alpha * M_list_class[ii].angle + (1-alpha)*Mo.angle

    M_msg = making_M(M_list_class)
    OM_publisher.publish(M_msg)
    if len(data.M_list)>0:
        print "Done"


# Initial node:
rospy.init_node('List_of_mapped_objects', anonymous=True)


global M_list_class
global epsilon
epsilon = rospy.get_param('/Cov/epsilon')
M_list_class = []
# Publishers:
OM_publisher = rospy.Publisher('/M',M,queue_size=5)

while not rospy.is_shutdown():

    Theta_list_sub = rospy.Subscriber('/M_o',M_Suggested_List,M_o_callback,queue_size=20)
    rospy.wait_for_message('/M_o',M_Suggested_List)

 
   


