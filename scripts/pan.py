#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
import json


class Pan:
    def __init__(self):

        self.state = "center"
        self.inst_count = 0
        self.response_pub = ""
        self.pose_pub = ""
        self.current_position_val = 0
        self.current_velocity_val = 0

        print "Pan Node has been initialized"

    def change_pan_direction(self, direction):
        print "Received pan direction: ", direction
        if direction == "center":
            self.pose_pub.publish(0)
            while (not abs(round(self.current_position_val, 2)) <= 0.05
                   or abs(round(self.current_velocity_val, 1)) == 0
                   and not rospy.is_shutdown()):
                if self.current_position_val > .1:
                    self.pose_pub.publish(-.1)
                elif self.current_position_val < -.1:
                    self.pose_pub.publish(.1)
                elif self.current_position_val > .01:
                    self.pose_pub.publish(-5 * self.current_velocity_val)
                elif self.current_position_val < -.01:
                    self.pose_pub.publish(-5 * self.current_velocity_val)
                else:
                    self.pose_pub.publish(-10 * self.current_velocity_val)
        elif direction == "left":
            if (not (round(self.current_position_val, 1) >= 1.5)):
                self.pose_pub.publish(1.5)
            while (not round(self.current_position_val, 1) >= 1.5
                   and not rospy.is_shutdown()):
                pass
        elif direction == "right":
            if (not (round(self.current_position_val, 1) <= -1.5)):
                self.pose_pub.publish(-1.5)
            while (not (round(self.current_position_val, 1) <= -1.5)
                   and not rospy.is_shutdown()):
                print round(self.current_position_val, 1)
                pass

        print "Done panning"

    def callback(self, data):
        parsed_json = json.loads(data.data)
        self.inst_count = parsed_json["inst_count"]
        instruction = parsed_json["direction"]
        self.set_state(instruction)
        if instruction == "center":
            self.change_pan_direction("center")
            self.publish_response()
        elif instruction == "left":
            self.change_pan_direction("left")
            self.publish_response()
        elif instruction == "right":
            self.change_pan_direction("right")
            self.publish_response()

        print "State changed to ", self.state

    def setup(self):

        rospy.init_node('pan', anonymous=True)

        rospy.Subscriber('pan', String, self.callback)

        self.response_pub = rospy.Publisher('sensors', String, queue_size=10)

        self.pose_pub = rospy.Publisher(
            "/robot/pan_tilt_position_controller/command", Float64, queue_size=10)

        # Receive joint position information
        rospy.Subscriber('/robot/joint_states', JointState,
                         self.joint_states_callback)

        while not rospy.is_shutdown():
            if self.state == "center":
                if self.current_position_val > .1:
                    self.pose_pub.publish(-.1)
                elif self.current_position_val < -.1:
                    self.pose_pub.publish(.1)
                elif self.current_position_val > .01:
                    self.pose_pub.publish(-5 * self.current_velocity_val)
                elif self.current_position_val < -.01:
                    self.pose_pub.publish(-5 * self.current_velocity_val)
                else:
                    self.pose_pub.publish(-10 * self.current_velocity_val)

    def joint_states_callback(self, data):
        self.current_position_val = data.position[4]
        self.current_velocity_val = data.velocity[4]
        # print "Pan position: ", self.current_position_val

    def get_state(self):
        return self.state

    def set_state(self, new_state):
        self.state = new_state

    def pan_left(self):
        video_dir.home_x_y()
        time.sleep(0.5)
        for i in range(0, 8):
            video_dir.move_decrease_x()
        time.sleep(1)

    def pan_right(self):
        video_dir.home_x_y()
        time.sleep(0.5)
        for i in range(0, 8):
            video_dir.move_increase_x()
        time.sleep(1)

    def publish_response(self):
        self.response_pub.publish(json.dumps({
            "sensor": "pan",
            "inst_count": self.inst_count
        }))


if __name__ == '__main__':
    try:
        pan = Pan()
        pan.setup()
    except rospy.ROSInterruptException:
        pass
