#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

Move = [0,0,0,0]

def callback(joy):
  global Move
  Move  = [joy.buttons[4],-joy.buttons[0],joy.buttons[3],-joy.buttons[1]]
  #print "pose callback"

def main ():
  global Move
  
  try: 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('/joy', Joy, callback)
    rospy.init_node('keyboard', anonymous=True)
    
    rate = rospy.Rate(90)

    while not rospy.is_shutdown():
      #left = Float64()
      #right = Float64()
      vel  = Twist()

      #left.data = (Move[0]+Move[1])*3
      #right.data = (Move[2]+Move[3])*3
      #left.data = (Move[0] - Move[3] + Move[1]) * 3
      #right.data = (Move[0] + Move[2] + Move[1]) * 3
      vel.linear.x = (Move[0] + Move[1]) * 0.3
      vel.angular.z = (Move[2] + Move[3])*0.3
      pub.publish(vel)
      
      #pub_left.publish(left)
      #pub_right.publish(right)

      
      rate.sleep()
      #left.data = 0; right.data = 0
      #pub_left.publish(left)
      #pub_right.publish(right)
    rospy.spin()
    
  except Exception as e:
    print(e)
  
  finally:
    #left.data = 0; right.data = 0
    #pub_left.publish(left)
    #pub_right.publish(right)
    vel.linear.x = 0; vel.angular.z = 0
    vel.linear.y = 0; vel.linear.z = 0; vel.angular.x = 0; vel.angular.y = 0
    pub.publish(vel)

if __name__=="__main__":
  
  #try:
    main()
  #except rospy.ROSInterruptException:
   #pass
