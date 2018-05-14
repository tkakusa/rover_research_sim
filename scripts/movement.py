#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import sys
import serial
from rover_research.srv import *

import math
import time
import json

# Default values
ROTATION_SPEED = 40
SLOW_ROTATION_SPEED = 30
FORWARD_SPEED = .2
REST_TIME = 1


class Movement:
    def __init__(self):

        self.response_pub = ""
        self.movement_pub = ""
        self.orientation_sub = ""
        self.response_ser = ""

        self.state = "stop"
        self.rotationAngle = 0
        self.inst_count = 0
        self.current_bearing = 0

        # Speed setting
        self.forward_speed = 0.2
        self.normal_forward_speed = 0.1
        self.turning_speed = 0.6
        self.slight_turning_speed = 0.1

        print "Movement Node has been initialized"

    def callback(self, data):
        received = json.loads(data.data)
        if received["direction"] == "forward":
            self.state = "forward"
        elif received["direction"] == "backward":
            self.state = "backward"
        elif received["direction"] == "left":
            if received["rotation"]:
                self.rotationAngle = -received["rotation"]
                self.inst_count = received["inst_count"]
                self.state = "rotation"
            else:
                self.state = "left"
        elif received["direction"] == "right":
            if received["rotation"]:
                self.rotationAngle = received["rotation"]
                self.inst_count = received["inst_count"]
                self.state = "rotation"
            else:
                self.state = "right"
        elif received["direction"] == "slight_left":
            self.state = "slight_left"
        elif received["direction"] == "slight_right":
            self.state = "slight_right"
        elif received["direction"] == "stop":
            self.state = "stop"

        print("State changed to ", self.state)

    def imu_callback(self, data):
        imu_data = data.orientation
        orientation_list = [imu_data.x, imu_data.y, imu_data.z, imu_data.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        degrees = yaw * 180 / math.pi
        if degrees < 0:
            degrees = 360 + degrees
        self.current_bearing = 360 - degrees
        #print 360 - self.current_bearing

    def setup(self):

        rospy.init_node('mover', anonymous=True)

        rospy.Subscriber("movement", String, self.callback)

        self.response_pub = rospy.Publisher('sensors', String, queue_size=10)

        # Publisher for the cmd_vel movement commands
        self.movement_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscriber for IMU messages
        rospy.Subscriber('imu', Imu, self.imu_callback)

        # Create a service for retrieving information
        self.response_ser = rospy.Service(
            "get_movement_data", SimpleString, self.handle_get_movement_data)
        self.set_service = rospy.Service(
            "set_movement_data", SetData, self.handle_set_movement_data)

        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()

    def handle_get_movement_data(self, req):
        if req.str == "state":
            return SimpleStringResponse(self.state)

    def handle_set_movement_data(self, req):
        if req.set_param == "speed":
            OldRange = (100 - 30)
            NewRange = (.6 - .05)
            NewValue = (((req.value - 30) * NewRange) / OldRange) + .05
            self.forward_speed = NewValue
            return SetDataResponse()

    def get_state(self):
        return self.state

    def set_state(self, new_state):
        self.state = new_state

    def turnTheRobot(self):
        currentBearing = self.current_bearing
        newBearing = currentBearing + self.rotationAngle
        if newBearing < 0:
            newBearing += 360
        elif newBearing > 360:
            newBearing -= 360
        print "Current Bearing: ", currentBearing
        print "New Bearing:     ", newBearing
        while self.state == "rotation" and not rospy.is_shutdown():
            currentBearing = self.current_bearing
            error = currentBearing - newBearing
            #print currentBearing
            if math.fabs(error) < 2:
                self.move_stop()
                self.response_pub.publish(json.dumps({
                    "sensor": "movement",
                    "inst_count": self.inst_count
                }))
                break
            if error > 180:
                error -= 360
            if error < -180:
                error += 360

            # Adjust speed according to how far from angle desired
            if math.fabs(error) < 20:
                if error > 0:
                    self.turn_left('slow')
                else:
                    self.turn_right('slow')
            else:
                if error > 0:
                    self.turn_left('normal')
                else:
                    self.turn_right('normal')

            # print(currentBearing)
            time.sleep(.1)

    def move_stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.movement_pub.publish(vel_msg)

    def move_forward(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.forward_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.movement_pub.publish(vel_msg)

    def move_backward(self):
        vel_msg = Twist()
        vel_msg.linear.x = -self.forward_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.movement_pub.publish(vel_msg)

    def slight_left(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.forward_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -self.slight_turning_speed
        self.movement_pub.publish(vel_msg)

    def turn_left(self, speed):
        vel_msg = Twist()
        if speed == "slow":
            vel_msg.linear.x = 0.05
        else:
            vel_msg.linear.x = self.normal_forward_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -self.turning_speed
        self.movement_pub.publish(vel_msg)

    def slight_right(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.forward_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.slight_turning_speed
        self.movement_pub.publish(vel_msg)

    def turn_right(self, speed):
        vel_msg = Twist()
        if speed == "slow":
            vel_msg.linear.x = 0.05
        else:
            vel_msg.linear.x = self.normal_forward_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.turning_speed
        self.movement_pub.publish(vel_msg)


if __name__ == '__main__':
    movement = Movement()
    movement.setup()
    rate = rospy.Rate(10)
    bearing = 0
    while not rospy.is_shutdown():
        if movement.get_state() == "stop":
            movement.move_stop()
            movement.set_state("wait")
        elif movement.get_state() == "forward":
            movement.move_forward()
            movement.set_state("straight_forward")
        elif movement.get_state() == "straight_forward":
            pass
        elif movement.get_state() == "backward":
            movement.move_backward()
            movement.set_state("straight_backward")
        elif movement.get_state() == "straight_backward":
            pass
        elif movement.get_state() == "left":
            movement.turn_left()
            movement.set_state("wait")
        elif movement.get_state() == "right":
            movement.turn_right()
            movement.set_state("wait")
        elif movement.get_state() == "slight_left":
            movement.slight_left()
            movement.set_state("wait")
        elif movement.get_state() == "slight_right":
            movement.slight_right()
            movement.set_state("wait")
        elif movement.get_state() == "rotation":
            movement.turnTheRobot()
            if movement.get_state() == "rotation":
                movement.set_state("wait")
        rate.sleep()
    print ("Exited")
