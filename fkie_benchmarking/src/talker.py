#!/usr/bin/env python
import os
import random
import string
import rospy
from std_msgs.msg import String

def get_random_string():
    return ''.join(random.sample(string.lowercase, 5))

def talker():
    topic_name_prefix = os.getenv('NATIVE_NETWORK_INSTANCE', 'chatter')
    topic_num = int(os.getenv('TOPIC_NUM', '5'))
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = []
    for i in range(topic_num):
        rand_str = get_random_string()
        pub.append(rospy.Publisher(topic_name_prefix+'-'+rand_str, String, queue_size=10))
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        for i in range(topic_num):
            pub[i].publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
