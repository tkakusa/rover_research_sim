#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pickle
import socket
from socket import error as socket_error
import json
import copy
import select
import pdb
from rover_research.srv import *


class MessageBroker:
    '''
    def __init__(self)

    Description: Initialization for the MessageBroker class

    Inputs:      None

    Outputs:     None
    '''

    def __init__(self):
        # Initialize the socket information
        host = ''
        port = 8080
        backlog = 5
        size = 1024
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host, port))
        self.s.listen(backlog)
        self.socket_list = [self.s]

        # Error parameters
        self.FORWARD_COLISION_DISTANCE = 20
        self.BACKWARD_COLISION_DISTANCE = 30
        self.TURN_COLISION_DISTANCE = 5

        # Instruction location variables
        self.curr_inst = 0
        self.next_inst = 0

        # Sets up the default values for the variables
        self.ultra_distance = 100
        self.front_distance = 100
        self.back_distance = 100
        self.left_distance = 100
        self.right_distance = 100
        self.distance_adjustment = 10
        self.measurement_value = 30

        # Detectable objects
        self.orange_ball = False

    def next_instruction(self, curr, next):
        found = False
        self.curr_inst = curr
        self.next_inst = next
        # Load the current instruction
        inst = self.instructions[curr]
        instructions = copy.deepcopy(self.instructions)
        # Check that the running instruction list is not empty
        if len(self.running_instructions):
            # Clear the entire running instruction array if loop is done
            if inst["inst_type"] == "loop":
                print "Cleared instructions"
                self.running_instructions = []
            # Remove the current instruction when done
            else:
                for instruction in self.running_instructions:
                    if instruction["inst_count"] == inst["inst_count"]:
                        self.running_instructions.remove(instruction)
                        print "Removed instruction ", inst["inst_count"]
                        break

        # Check to make sure it is possible to move to the next instruction
        if next >= len(self.instructions):
            print "Finished"
            self.finished = True
            return

        # Make sure not to have copies of the same instructions
        if self.instructions[next] in self.running_instructions:
            print "Already in the list"
            return

        # Check to see if the next instruction is a loop or check instruction
        if ((self.instructions[next]["inst_type"] == "check")
            or (self.instructions[next]["inst_type"] == "check_alt")
                or (self.instructions[next]["inst_type"] == "loop")):
            # Check the parameters for the instruction
            parameters = self.instructions[next]["parameters"]
            instruction = self.instructions[next]
            next = instruction["inst_next"]
            distance = self.front_distance
            # Check if the statement is true
            if self.compare(parameters["param_1"],
                            parameters["param_2"],
                            parameters["equality"]):
                # Go to the next instruction
                self.curr_inst = instruction["inst_count"]
                self.next_inst = next
                # self.next_instruction(instruction["inst_count"], next)
            # If false
            else:
                # Go to the fail next instruction
                next = instruction["inst_fail_next"]
                self.curr_inst = instruction["inst_count"]
                self.next_inst = next
                # self.next_instruction(instruction["inst_count"], next)

        # Check if the next instruction is a loop_wait instruction
        # and keep looping if so
        elif self.instructions[next]["inst_type"] == "loop_wait":
            # Check the parameters for the instruction
            parameters = self.instructions[next]["parameters"]
            instruction = self.instructions[next]
            next = instruction["inst_next"]
            # distance = self.front_distance

            # Loop while the statement is true
            while (not self.compare(parameters["param_1"],
                                    parameters["param_2"],
                                    parameters["equality"])
                   and not rospy.is_shutdown()):
                pass

            # Once the looping is done
            next = instruction["inst_fail_next"]
            self.curr_inst = instruction["inst_count"]
            self.next_inst = next
            # self.next_instruction(instruction["inst_count"], next)

        elif self.instructions[next]["inst_type"] == "get":
            parameters = self.instructions[next]["parameters"]
            if parameters["get_param"] == "left_dist":
                self.left_distance = self.get_node_data("ultrasonic", "pan")
            elif parameters["get_param"] == "right_dist":
                self.right_distance = self.get_node_data("ultrasonic", "pan")
            instruction = instructions[next]
            next = instruction["inst_next"]
            self.curr_inst = instruction["inst_count"]
            self.next_inst = next
            # self.next_instruction(instruction["inst_count"], next)
        elif self.instructions[next]["inst_type"] == "set":
            parameters = self.instructions[next]["parameters"]
            self.set_node_data(
                parameters["node"], parameters["set_param"], parameters["value"])
            instruction = instructions[next]
            next = instruction["inst_next"]
            self.curr_inst = instruction["inst_count"]
            self.next_inst = next
            # self.next_instruction(instruction["inst_count"], next)
        else:
            instruction = instructions[next]
            self.running_instructions.append(instructions[next])
            self.pub.publish(json.dumps(instructions[next]))

    def sensor_callback(self, data):
        changed = False
        json_string = data.data
        json_form = json.loads(json_string)
        if json_form["sensor"] == "ultrasonic":
            self.front_distance = json_form["front_distance"]
            self.back_distance = json_form["back_distance"]
            for inst in self.running_instructions:
                count = 0
                parameters = inst["parameters"]
                if "ultrasonic" in parameters:
                    if parameters["ultrasonic"]["status"] == "active":
                        if self.compare(parameters["param_1"], parameters["param_2"], parameters["equality"]):
                            parameters["ultrasonic"]["achieved"] = True
        elif json_form["sensor"] == "timer":
            for inst in self.running_instructions:
                count = 0
                if inst["inst_count"] == json_form["inst_count"]:
                    print "Timer Returned"
                    parameters = inst["parameters"]
                    parameters["time"]["achieved"] = True

        elif json_form["sensor"] == "movement":
            for inst in self.running_instructions:
                count = 0
                if inst["inst_count"] == json_form["inst_count"]:
                    print "Movement Returned"
                    parameters = inst["parameters"]
                    parameters["movement"]["achieved"] = True

        elif json_form["sensor"] == "pan":
            for inst in self.running_instructions:
                count = 0
                if inst["inst_count"] == json_form["inst_count"]:
                    print "Pan Returned"
                    parameters = inst["parameters"]
                    parameters["pan"]["achieved"] = True

        elif json_form["sensor"] == "ball_tracking":
            print json_form
            self.orange_ball = json_form["orange_ball"]

    # Compare two parameters for check and loop instructions
    def compare(self, param_1, param_2, equality):
        # Identify the first parameter
        if param_1 == "distance":
            # Check the current direction of the movement
            current_direction = self.get_node_data("movement", "state")
            if current_direction == "backward" or current_direction == "straight_backward":
                p1 = self.back_distance
            else:
                p1 = self.front_distance
        elif param_1 == "left_dist":
            p1 = self.left_distance
        elif param_1 == "right_dist":
            p1 = self.right_distance
        elif param_1 == "orange_ball":
            p1 = self.orange_ball

        # Identify the second parameters
        if param_2 == "distance":
            p2 = self.front_distance
        elif param_2 == "left_dist":
            p2 = self.left_distance
        elif param_2 == "right_dist":
            p2 = self.right_distance
        elif param_2 == "measurement":
            p2 = self.measurement_value
        else:
            p2 = param_2

        # Print the parameters
        # print "Parameter 1: ", p1
        # print "Parameter 2: ", p2

        # Check the equality
        if equality == "equal":
            return p1 == p2
        elif equality == "not equal":
            return p1 != p2
        elif equality == "more":
            return p1 > p2
        elif equality == "less":
            # print "Result: ", p1 < p2
            return p1 < p2

    # Get data specifically from another node
    def get_node_data(self, node_name, parameter):
        if node_name == "movement":
            rospy.wait_for_service('get_movement_data')
            try:
                movement_data = rospy.ServiceProxy(
                    'get_movement_data', SimpleString)
                resp1 = movement_data(parameter)
                return resp1.str
            except rospy.ServiceException, e:
                print "Service call failed"
                return -1
        if node_name == "ultrasonic":
            rospy.wait_for_service('get_ultrasonic_data')
            try:
                ultrasonic_data = rospy.ServiceProxy(
                    'get_ultrasonic_data', StringInt)
                resp1 = ultrasonic_data(parameter)
                return resp1.dist
            except rospy.ServiceException, e:
                print "Service call failed"
                return -1

    # Set specific data in another node
    def set_node_data(self, node_name, set_param, value):
        if node_name == "messageBroker":
            if set_param == "measurement":
                if str(value).isdigit():
                    self.measurement_value = value
                elif value == 'front_distance':
                    self.measurement_value = self.front_distance
                elif value == 'left_distance':
                    self.measurement_value = self.left_distance
                elif value == 'right_distance':
                    self.measurement_value = self.right_distance
                elif value == 'back_distance':
                    self.measurement_value = self.back_distance
                else:
                    print "Error: Invalid measurement value to set"
                    return -1
                print "Measurement value set to ", self.measurement_value
                return
        if node_name == "movement":
            rospy.wait_for_service('set_movement_data')
            try:
                movement_data = rospy.ServiceProxy(
                    'set_movement_data', SetData)
                movement_data(set_param, value)
                return
            except rospy.ServiceException, e:
                print "Service call failed"
                return -1
        print "Error: Invalid node name: ", node_name
        return -1

    # Check for errors
    def error_check(self):
        # Check if a collision could happen
        current_direction = self.get_node_data("movement", "state")
        if current_direction == "forward" or current_direction == "straight_forward":
            if self.front_distance <= self.FORWARD_COLISION_DISTANCE:
                print "ERROR imminent forward collision detected... All actions have been stopped"
                self.finished = True
        elif current_direction == "left" or current_direction == "right" or current_direction == "rotation":
            if self.front_distance <= self.TURN_COLISION_DISTANCE:
                print "ERROR imminent turning collision detected... All actions have been stopped"
                self.finished = True
        elif current_direction == "backward" or current_direction == "straight_backward":
            # print("Back Distance: ", self.back_distance)
            if self.back_distance <= self.BACKWARD_COLISION_DISTANCE:
                print "ERROR imminent backward collision detected... All actions have been stopped"
                self.finished = True

    # Close all socket connections
    def close_sockets(self):
        for s in self.socket_list:
            s.close()

    def messageBroker(self):

        self.pub = rospy.Publisher('instruction', String, queue_size=10)
        self.status_pub = rospy.Publisher('status', String, queue_size=10)
        rospy.init_node('messageBroker', anonymous=True)
        rospy.Subscriber('sensors', String, self.sensor_callback)

        self.status_pub.publish("stop")
        while not rospy.is_shutdown():
            self.finished = False
            self.instructions = []
            self.running_instructions = []
            print 'Waiting for connection...'
            client, addr = self.s.accept()
            print '...connected from :', addr
            while True and not rospy.is_shutdown():
                data = client.recv(1024)
                try:
                    data = pickle.loads(str(data))
                    if data == "Done":
                        print("Entered the done state")
                        break
                    if not data == "Done":
                        self.instructions.append(data)
                        # print(data)
                except:
                    print(data)
                    break
            print "Instructions received, starting program..."
            print "Size of instrunctions = ", len(self.instructions)
            self.running_instructions.append(self.instructions[0])
            self.status_pub.publish("start")
            # rospy.sleep(3)
            self.next_instruction(0, 0)
            try:
                # Add the client to the socket list
                self.socket_list.append(client)
                self.curr_inst = 0
                self.next_inst = 0
                while not self.finished and not rospy.is_shutdown():
                    if len(self.running_instructions) is 0:
                        self.next_instruction(self.curr_inst, self.next_inst)
                    # Loop through the instructions being run
                    for inst in self.running_instructions:
                        count = 0
                        parameters = inst["parameters"]
                        for key, value in parameters.items():
                            if not key == "count":
                                try:
                                    if parameters[key]["achieved"]:
                                        count += 1
                                except TypeError as e:
                                    print e
                                    print parameters
                                    exit()
                        if count == parameters["count"]:
                            if inst["inst_type"] == "loop":
                                self.next_instruction(
                                    inst["inst_count"], inst["inst_fail_next"])
                            else:
                                self.next_instruction(
                                    inst["inst_count"], inst["inst_next"])

                    # Check for any messages from the client
                    # This line makes it so that the accept function below is non blocking
                    readable, writable, errored = select.select(
                        self.socket_list, [], [], 0)

                    # Search through all the readable sockets to see the type it is
                    for sockets in readable:
                        # If it is the host socket search for new connections
                        if sockets == self.s:
                            client, address = self.s.accept()
                            print("New Device Added")

                            # Add the new Connection
                            socket_list.append(client)
                        # Incoming message detection
                        if not sockets == self.s:
                            data = sockets.recv(4096)
                            try:
                                data = pickle.loads(str(data))
                                if data == "stop":
                                    print "ERROR Stop message has been received"
                                    print "Stopping code execution"
                                    # pdb.set_trace()
                                    self.socket_list.remove(sockets)
                                    self.next_instruction(
                                        self.instructions[0]["inst_count"], self.instructions[-1])
                                else:
                                    print(data)
                                    pass
                            except EOFError:
                                break

                    # Check for any problems or errors that could occur during runtime
                    self.error_check()

            except KeyboardInterrupt:
                pass

            # Tell all of the nodes to stop
            self.status_pub.publish("stop")

            # Close client socket
            for s in self.socket_list:
                if not s == self.s:
                    try:
                        s.send(pickle.dumps("stop"))
                    except:
                        pass
                    self.socket_list.remove(s)


if __name__ == '__main__':
    print("started")
    brocker = MessageBroker()
    try:
        brocker.messageBroker()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        brocker.close_sockets()
        print "Exiting Code"
    except socket_error as e:
        brocker.close_sockets()
        print e
        print "Exiting Code"
