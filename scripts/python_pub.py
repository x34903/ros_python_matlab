#!/usr/bin/env python

import rospy #allow ROS interface
from std_msgs.msg import Float64MultiArray #ROS data format
import numpy as np #matrix operations


# SET UP PUBLISHERS
data_to_pub = Float64MultiArray()
pub = rospy.Publisher('Aflat_py', Float64MultiArray, queue_size=10)


def python_pub():

    # set up ROS node
    rospy.init_node('python_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        # SOME MATRIX TO PUBLISH
        A = np.array([ [1,2,3,4] , [5,6,7,8] , [9,10,11,12] , [13,14,15,rospy.get_time()] ])


        # PREPROCESS AND PUBLISH 
        data_to_pub.data = np.reshape(A,16)  
        pub.publish(data_to_pub)


        # SLOW THIS DOWN AS REQUIRED        
        rate.sleep()

if __name__ == '__main__':
    try:
        python_pub()
    except rospy.ROSInterruptException:
        pass
