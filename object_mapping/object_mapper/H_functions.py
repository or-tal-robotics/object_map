#!/usr/bin/env python

# Libraries:
import rospy
import numpy as np
import pandas as pd
from scipy.stats import multivariate_normal as mn
 

# Msgs:
from object_mapping.msg import Single_Class , M_i , M

# Yamls:
A_Round = rospy.get_param('/Array/round')

def q_i(mi,mo):
    global A_Round
    q_i = 0
    for ii in range(0,12):
        if mi.cls_num[ii] in A_Round:
            theta_i = np.array([mi.x_center[ii],mi.y_center[ii],mi.r[ii]])
            theta_o = np.array([mo.x_center[ii],mo.y_center[ii],mo.r[ii]])
        else:
            theta_i = np.array([mi.x_center[ii],mi.y_center[ii],mi.angle[ii],mi.a[ii],mi.b[ii]])
            theta_o = np.array([mo.x_center[ii],mo.y_center[ii],mo.angle[ii],mo.a[ii],mi.b[ii]])
        
        sigma = np.array(rospy.get_param('/Cov/o' + str(mi.cls_num[ii])))
        f_mi_mo = mn.pdf(theta_o, mean = theta_i ,cov = sigma)

        q_i += mi.prob_distribution[ii] * mo.prob_distribution[ii] * f_mi_mo
    return q_i

def _Making_M_i(mi):
    
    M_i_list = M_i()
    for i in range(0,12):
        SO = Single_Class()
        SO.x_center =  mi.x_center[i]
        SO.y_center =  mi.y_center[i]
        SO.r = mi.r[i]
        SO.a = mi.a[i]
        SO.b = mi.b[i]
        SO.angle = mi.angle[i]
        SO.cls_num = mi.cls_num[i]
        SO.probability = mi.prob_distribution[i]
        M_i_list.m_i.append(SO)
    return M_i_list

def making_M(M_class_list):

    M_msg_list = M()
    for ii in range (0,len(M_class_list)):
        M_msg_list.M.append(_Making_M_i(M_class_list[ii]))

    return M_msg_list

def Prob_updater(alpha,P_old,P_new):

    P_star = (P_old**alpha) * (P_new**(1-alpha))
    P_sum = np.sum(P_star)
    P = P_star / P_sum
    return P

# Single object class:
class SO_class():

    def __init__(self,x_center=[],y_center=[],r=[],a=[],b=[],angle=[],cls_num=[],prob_distribution=[]):

        self.x_center = np.array(x_center)
        self.y_center = np.array(y_center)
        self.r = np.array(r)
        self.a = np.array(a)
        self.b = np.array(b)
        self.angle = np.array(angle)
        self.cls_num = np.array(cls_num)
        self.prob_distribution = np.array(prob_distribution)



