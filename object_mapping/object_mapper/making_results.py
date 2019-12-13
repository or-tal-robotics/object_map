#!/usr/bin/env python
import rospy
import numpy as np
import xlsxwriter 

from object_mapping.msg import M
from gazebo_msgs.msg import ModelStates

global x_model , y_model , name


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

M_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,callback_model)
rospy.wait_for_message('/gazebo/model_states',ModelStates)


print "Start Model_table xls!"
workbook = xlsxwriter.Workbook('Model_table_smaller_eps.xlsx')
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
for num in x_model:
    worksheet.write(row, column, num)
    column += 1
row += 1
column = 0

worksheet.write(row, column, 'y')
column += 1

for num in y_model:
    worksheet.write(row, column, num)
    column += 1
row += 1
column = 0



workbook.close()

print "Done Model_table xls!"