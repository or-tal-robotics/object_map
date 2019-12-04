#!/usr/bin/env python
import rospy
import numpy as np
import xlsxwriter 

from object_mapping.msg import M
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

global x_world,y_world,x_map,y_map,x_model , y_model , name


def callback_world(data):
    global x_world,y_world
    [x_world,y_world] = [data.pose.pose.position.x , data.pose.pose.position.y]

def callback_pose(data):
    global x_map,y_map
    [x_map,y_map] = [data.pose.position.x,data.pose.position.y]

def callback_model(data):

    global x_model , y_model , name
    name = data.name
    x_model = []
    y_model = []
    for ii in range(0,len(name)):
        x_model.append(data.pose[ii].position.x)
        y_model.append(data.pose[ii].position.y)
    x_model = np.array(x_model)
    y_model = np.array(y_model)

rospy.init_node('xls_MObjects_node', anonymous=True)

W_sub = rospy.Subscriber('/odom',Odometry,callback_world)
rospy.wait_for_message('/odom',Odometry)
P_sub = rospy.Subscriber('/slam_out_pose',PoseStamped,callback_pose)
rospy.wait_for_message('/slam_out_pose',PoseStamped)
M_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,callback_model)
rospy.wait_for_message('/slam_out_pose',ModelStates)

x_cor = x_map - x_world
y_cor = y_map - y_world
x_m = x_model +  x_cor
y_m = y_model + y_cor

print "Start Model_table xls!"
workbook = xlsxwriter.Workbook('Model_table_third_time.xlsx')
#workbook = xlsxwriter.Workbook('Model_table.xlsx')
worksheet = workbook.add_worksheet()
row = 0
column = 0

worksheet.write(row, column, 'model_name')
column += 1

for n in name:

    worksheet.write(row, column, n)
    column += 1
row += 1
column = 0

worksheet.write(row, column, 'x')
column += 1
for num in x_m:
    worksheet.write(row, column, num)
    column += 1
row += 1
column = 0

worksheet.write(row, column, 'y')
column += 1

for num in y_m:
    worksheet.write(row, column, num)
    column += 1
row += 1
column = 0



workbook.close()

print "Done Model_table xls!"