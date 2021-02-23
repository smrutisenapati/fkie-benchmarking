#!/usr/bin/env python
import os
import random
import string
import rospy
from std_msgs.msg import String

def get_random_string(len):
    return ''.join(random.sample(string.lowercase, len))

def talker():
    start = int(os.getenv('START', '1'))
    end = int(os.getenv('END', '5'))
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = []
    for i in range(start, end+1):
        pub.append(rospy.Publisher('chatter{}'.format(i), String, queue_size=10))
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        for i in range(end-start+1):
            pub[i].publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
