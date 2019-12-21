#!/usr/bin/env python

import rospy
import xlsxwriter 

from object_mapping.msg import M

def callback_map(data):
    global O_map
    O_map = data
    exp_num = input ('Experiment number: ')
    num = input ('Round number: ')
    
    if num == 1:
        workbook_name = 'new_Results' + str(exp_num) + '.xlsx'
    else:
        workbook_name = 'new_Results' + str(exp_num) + '_second_round.xlsx'
    workbook = xlsxwriter.Workbook(workbook_name)
    worksheet = workbook.add_worksheet()
    row = 0
    column = 0

    for ii in range(0,len(O_map.M)):
        
        names = [str(ii),
                'bicycle','bird','bottle','cat','Chair',
                'dog','motorbike','person','pottedplant',
                'sheep','sofa','tvmonitor']
        
        
        x = []
        y = []
        probabilities = []
        x_max = 0
        y_max = 0
        prob_max = 0

        for jj in range(0,len(O_map.M[ii].m_i)):
            x.append(O_map.M[ii].m_i[jj].x_center)
            y.append(O_map.M[ii].m_i[jj].y_center)
            probabilities.append(O_map.M[ii].m_i[jj].probability)
            if O_map.M[ii].m_i[jj].probability > prob_max:
                x_max= O_map.M[ii].m_i[jj].x_center
                y_max = O_map.M[ii].m_i[jj].y_center
                prob_max= O_map.M[ii].m_i[jj].probability
                name_max = names[jj+1]
        
        for name in names:

            worksheet.write(row, column, name)
            column += 1
        column += 2
        worksheet.write(row, column, name_max)
        row += 1
        column = 0

        worksheet.write(row, column, 'x')
        column += 1
        for num in x:
            worksheet.write(row, column, num)
            column += 1
        column += 2
        worksheet.write(row, column, x_max) 
        row += 1
        column = 0

        worksheet.write(row, column, 'y')
        column += 1
        for num in y:
            worksheet.write(row, column, num)
            column += 1
        column += 2
        worksheet.write(row, column, y_max) 
        row += 1
        column = 0

        worksheet.write(row, column, 'prob')
        column += 1
        for num in probabilities:
            worksheet.write(row, column, num)
            column += 1
        column += 2
        worksheet.write(row, column, prob_max) 
        row += 2
        column = 0


    workbook.close()

rospy.init_node('xls_Objects_node', anonymous=True)


#Subsctibers:
M_Subs = rospy.Subscriber('/M',M,callback_map)
rospy.wait_for_message('/M',M)


