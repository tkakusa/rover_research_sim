#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json


class Arbiter():
    def __init__(self):
        self.movement_pub = ""
        self.ultrasonic_pub = ""
        self.timer_pub = ""
        self.pan_bub = ""

    def status_callback(self, data):
        if data.data == "start":
            self.ultrasonic_pub.publish("on")
        elif data.data == "stop":
            self.ultrasonic_pub.publish("off")
            self.movement_pub.publish(json.dumps({
                "direction": "stop"
            }))

    def instruction_callback(self, data):
        json_string = data.data
        json_form = json.loads(json_string)
        if "movement" in json_form:
            self.movement_pub.publish(json.dumps(json_form["movement"]))
        if "timer" in json_form:
            self.timer_pub.publish(json.dumps(json_form["timer"]))
        if "pan" in json_form:
            self.pan_pub.publish(json.dumps(json_form["pan"]))

    # def response_callback(self, data):
    #    self.movement_pub.publish("s")

    def setup(self):
        rospy.init_node('arbiter', anonymous=True)

        self.movement_pub = rospy.Publisher('movement', String, queue_size=10)
        self.timer_pub = rospy.Publisher('timer', String, queue_size=10)
        self.ultrasonic_pub = rospy.Publisher(
            'ultrasonic', String, queue_size=10)
        self.pan_pub = rospy.Publisher('pan', String, queue_size=10)

        rospy.Subscriber("instruction", String, self.instruction_callback)
        # rospy.Subscriber("response", String, self.response_callback)
        rospy.Subscriber("status", String, self.status_callback)

        print("Arbiter Node has been initialized")
        rospy.spin()


if __name__ == '__main__':
    try:
        arbiter = Arbiter()
        arbiter.setup()
    except rospy.ROSInterruptException:
        pass
