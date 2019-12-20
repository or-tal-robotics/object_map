#!/usr/bin/env python

import rospy

import xlsxwriter 

from object_mapping.msg import M

def callback_map(data):
    global O_map
    O_map = data
    workbook = xlsxwriter.Workbook('Results4_second_round.xlsx')
    #workbook = xlsxwriter.Workbook('Results4.xlsx')
    worksheet = workbook.add_worksheet()
    row = 0
    column = 0

    for ii in range(0,len(O_map.M)):
        
        names = [str(ii),'bicycle','bird','bottle','cat','Chair','dog','motorbike','person','pottedplant','sheep','sofa','tvmonitor']
        for name in names:

            worksheet.write(row, column, name)
            column += 1
        row += 1
        column = 0
        
        x = []
        y = []
        probabilities = []

        for jj in range(0,len(O_map.M[ii].m_i)):
            x.append(O_map.M[ii].m_i[jj].x_center)
            y.append(O_map.M[ii].m_i[jj].y_center)
            probabilities.append(O_map.M[ii].m_i[jj].probability)
        
        worksheet.write(row, column, 'x')
        column += 1
        for num in x:
            worksheet.write(row, column, num)
            column += 1
        row += 1
        column = 0

        worksheet.write(row, column, 'y')
        column += 1

        for num in y:
            worksheet.write(row, column, num)
            column += 1
        row += 1
        column = 0

        worksheet.write(row, column, 'prob')
        column += 1
        for num in probabilities:
            worksheet.write(row, column, num)
            column += 1
        row += 2
        column = 0

    workbook.close()

rospy.init_node('xls_Objects_node', anonymous=True)


#Subsctibers:
M_Subs = rospy.Subscriber('/M',M,callback_map)
rospy.wait_for_message('/M',M)


