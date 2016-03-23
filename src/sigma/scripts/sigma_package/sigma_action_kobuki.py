#! /usr/bin/env python

""" The MIT License (MIT)

    Copyright (c) 2016 Kyle Hollins Wray, University of Massachusetts

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""


import os
import sys

thisFilePath = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(thisFilePath, "..", "..", "libnova", "python"))
import nova.pomdp_alpha_vectors as pav

import rospy

from sigma.srv import GetAction
from sigma.srv import GetBelief
from sigma.srv import UpdateBelief


# The distaince at which we terminate saying that we're at the goal (in meters).
SIGMA_AT_GOAL_THRESHOLD = 0.01


class SigmaKobukiAction(object):
    """ A class to control a Kobuki following a POMDP policy. """

    def __init__(self):
        """ The constructor for the SigmaKobukiAction class. """

        self.x = 0
        self.y = 0
        self.theta = 0

        self.atGoal = False
        self.goalX = None
        self.goalY = None
        self.goalTheta = None

        sigmaPOMDPNamespace = rospy.get_param("~sigma_pomdp_namespace", "/sigma_pomdp_node")

        self.srvGetActionTopic = sigmaPOMDPNamespace + "/get_action"
        self.srvGetBeliefTopic = sigmaPOMDPNamespace + "/get_belief"
        self.srvUpdateBeliefTopic = sigmaPOMDPNamespace + "/update_belief"

    def initialize(self):
        """ Initialize the necessary variables to move the Kobuki, including messages. """

        # TODO: Setup the initial pose (location and theta) estimate.

        pass

    def update(self):
        """ Move the robot and handle sensing by interacting with the SigmaPOMDP node.

            Continue to move the robot, calling the SigmaPOMDP node, and read sensor data
            to update this POMDP's belief.
        """

        if not self.check_bump():
            self.move_robot()

    def move_robot(self):
        """ Move the robot based on the SigmaPOMDP node's action.

            This gets the current action, moves the robot, calls the update belief service,
            gets the next action upon arriving at the next location, etc. It does not
            handle interrupts via a bump.
        """

        if self.atGoal:
            # Service request for the SigmaPOMDP to send the current action.
            rospy.wait_for_service(self.srvGetActionTopic)

            srvGetAction = rospy.ServiceProxy(self.srvGetActionTopic, GetAction)

            self.goalX, self.goalY, self.goalTheta = srvGetAction()

            self.atGoal = False
        else:
            distanceToGoal = math.sqrt(pow(self.x - self.goalX, 2) +
                                       pow(self.y - self.goalY, 2))

            # If the robot is close to the goal, then clear the goal and send an observation.
            # Otherwise, move closer to the goal.
            if distanceToGoal < SIGMA_AT_GOAL_THRESHOLD:
                self.atGoal = True

                rospy.wait_for_service(self.srvUpdateBeliefTopic)
                srvUpdateBelief = rospy.ServiceProxy

    def check_bump(self):
        """ This method checks for sensing a bump. It also updates the belief.

            Returns:
                True if a bump was detected; False otherwise.
        """

        pass

