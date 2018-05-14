#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
import time
import json
import math
from rover_research.srv import *

front_ultrasonic_data = 0
back_ultrasonic_data = 0
pan_ultrasonic_data = 0


def callback(data):
    global state
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.data == "stop":
        state = "off"
    else:
        state = data.data
    rospy.loginfo(" State changed to " + state)
    """
    global state
    global sensor
    state = data.data
    if state == "on":
        sensor = PiMotor.Sensor("ULTRASONIC",10)
    rospy.loginfo(rospy.get_caller_id() + " State changed to " + state)
    """


def front_ultrasonic_callback(data):
    global front_ultrasonic_data
    front_ultrasonic_data = data.range / 0.0254


def back_ultrasonic_callback(data):
    global back_ultrasonic_data
    back_ultrasonic_data = data.range / 0.0254


def pan_ultrasonic_callback(data):
    global pan_ultrasonic_data
    pan_ultrasonic_data = data.range / 0.0254


def get_distance2():
    global front_ultrasonic_data
    global back_ultrasonic_data
    return (front_ultrasonic_data, back_ultrasonic_data)


def handle_get_ultrasonic_data(req):
    global pan_ultrasonic_data
    print "Received service request"
    if req.str == "pan":
        return StringIntResponse(pan_ultrasonic_data)


def setup():
    global sensor_pub

    time.sleep(0.1)

    rospy.init_node('ultrasonic', anonymous=True)

    rospy.Subscriber('ultrasonic', String, callback)

    rospy.Subscriber('front_ultrasonic', Range,
                     front_ultrasonic_callback, queue_size=10)
    rospy.Subscriber('back_ultrasonic', Range,
                     back_ultrasonic_callback, queue_size=10)
    rospy.Subscriber('pan_ultrasonic', Range,
                     pan_ultrasonic_callback, queue_size=10)

    # Create a service for retrieving information
    response_ser = rospy.Service(
        "get_ultrasonic_data", StringInt, handle_get_ultrasonic_data)

    sensor_pub = rospy.Publisher('sensors', String, queue_size=10)


if __name__ == '__main__':
    setup()
    global state
    global sensor
    global sensor_pub
    state = "off"
    rate = rospy.Rate(150)
    print("Entering State OFF")
    distance = 0
    previous = 0
    while not rospy.is_shutdown():
        rate.sleep()
        front_distances = []
        back_distances = []
        for i in range(0, 10):
            (front_distance, back_distance) = get_distance2()
            if front_distance != -1:
                front_distances.append(front_distance)
            if back_distance != -1:
                back_distances.append(back_distance)
            # rate.sleep()
        if len(front_distances) > 2 and len(back_distances) > 2:
            # Sort the readings
            front_distances.sort()
            back_distances.sort()

            # Remove the smallest
            front_distances.pop()
            back_distances.pop()

            # Reverse the data
            front_distances.reverse()
            back_distances.reverse()

            # Remove the largest
            front_distances.pop()
            back_distances.pop()

            # Get the average of the values
            front_distance = sum(front_distances) / len(front_distances)
            back_distance = sum(back_distances) / len(back_distances)
            if state == "on":
                sensor_pub.publish(json.dumps({
                    "sensor": "ultrasonic",
                    "front_distance": round(front_distance, 2),
                    "back_distance": round(back_distance, 2)
                }))
            else:
                pass
