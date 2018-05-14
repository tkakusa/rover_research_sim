#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time
import json
"""
state = "off"
timer = 0
response_pub = ""
"""

class TimerClass:
    def __init__(self, inst_count, time):
        print("Initializing the timer class")
        self.inst_count = inst_count
        rospy.Timer(rospy.Duration(time + 1), self.timercallback, oneshot=True)

    def timercallback(self, event):
        print ('Timer called for instance ', self.inst_count)
        response_pub.publish(json.dumps({
                                "sensor": "timer",
                                "inst_count": self.inst_count
                               }))


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    parsed_json = json.loads(data.data)
    if not parsed_json["time"] == "" :
        TimerClass(parsed_json["inst_count"], parsed_json["time"])

def setup():
    global response_pub

    rospy.init_node('timer', anonymous=True)

    rospy.Subscriber('timer', String, callback)

    response_pub = rospy.Publisher('sensors', String, queue_size=10)

    rospy.spin()
if __name__ == '__main__':
    setup()
