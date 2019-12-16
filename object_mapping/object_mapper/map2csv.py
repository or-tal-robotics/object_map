#!/usr/bin/env python

import rospy
import numpy as np
from object_mapping.msg import Object_Map
import matplotlib.pyplot as plt
import pandas as pd


def Map_callback(data):
    global Map_sub
    Map_sub = data


rospy.init_node('LaserMatrix', anonymous=True)
subscriber = rospy.Subscriber('/object_mapped_values',Object_Map, Map_callback)
rospy.wait_for_message('/object_mapped_values',Object_Map)


def M2C():
    global Map_sub
    Map = Map_sub.object_map
    df = pd.DataFrame(Map)
    df.to_csv('old_code_round_3.csv')

if __name__ == '__main__':
       
    #Testing our function
    M2C()