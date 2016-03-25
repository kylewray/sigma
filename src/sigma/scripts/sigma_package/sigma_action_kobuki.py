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

#from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent

from sigma.srv import GetAction
from sigma.srv import GetBelief
from sigma.srv import UpdateBelief

import math
import numpy as np


class SigmaActionKobuki(object):
    """ A class to control a Kobuki following a POMDP policy. """

    def __init__(self):
        """ The constructor for the SigmaActionKobuki class. """

        # These are the world-frame x, y, and theta values of the Kobuki. They
        # are updated as it moves toward the goal.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Again, these are world-frame goal values. Once it arrives at the goal, it will
        # get a new action, which assigns a new goal.
        self.atGoal = False
        self.goalX = 0.0
        self.goalY = 0.0
        self.goalTheta = 0.0

        # This bump variable is assigned in the callback for the sensor. It is used in
        # the odometry callback to control behavior.
        self.detectedBump = False

        # Setup the topics for the important services.
        sigmaPOMDPNamespace = rospy.get_param("~sigma_pomdp_namespace", "/sigma_pomdp_node")
        self.srvGetActionTopic = sigmaPOMDPNamespace + "/get_action"
        self.srvGetBeliefTopic = sigmaPOMDPNamespace + "/get_belief"
        self.srvUpdateBeliefTopic = sigmaPOMDPNamespace + "/update_belief"

        # The distance at which we terminate saying that we're at the goal,
        # in meters and radians, respectively.
        self.atPositionGoalThreshold = rospy.get_param("~at_position_goal_threshold", 0.05)
        self.atThetaGoalThreshold = rospy.get_param("~at_theta_goal_threshold", 0.05)

        # PID control variables.
        self.pidDerivator = 0.0
        self.pidIntegrator = 0.0
        self.pidIntegratorBounds = rospy.get_param("~pid_integrator_bounds", 0.05)

        # Load the gains for PID control.
        self.pidThetaKp = rospy.get_param("~pid_theta_Kp", 0.2)
        self.pidThetaKi = rospy.get_param("~pid_theta_Ki", 0.0)
        self.pidThetaKd = rospy.get_param("~pid_theta_Kd", 0.1)

        self.desiredVelocity = rospy.get_param("~desired_velocity", 0.2)

        # Finally, we create variables for the messages.
        self.started = False
        self.subKobukiOdom = None
        self.subKobukiBump = None
        self.pubKobukiVel = None

    def start(self):
        """ Start the necessary messages to operate the Kobuki. """

        if self.started:
            rospy.logwarn("Warn[SigmaActionKobuki.start]: Already started.")
            return

        subKobukiOdomTopic = rospy.get_param("~sub_kobuki_odom", "/odom")
        self.subKobukiOdom = rospy.Subscriber(subKobukiOdomTopic,
                                              Odometry,
                                              SigmaActionKobuki.sub_kobuki_odom,
                                              self)

        subKobukiBumpTopic = rospy.get_param("~sub_kobuki_bump", "/mobile_base/sensors/bumper")
        self.subKobukiBump = rospy.Subscriber(subKobukiBumpTopic,
                                              BumperEvent,
                                              SigmaActionKobuki.sub_kobuki_bump,
                                              self)

        pubKobukiVelTopic = rospy.get_param("~pub_kobuki_vel", "/cmd_vel")
        self.pubKobukiVel = rospy.Publisher(pubKobukiVelTopic, Twist, queue_size=32)

        self.started = True

    @staticmethod
    def sub_kobuki_odom(msg, self):
        """ Move the robot based on the SigmaPOMDP node's action.

            This gets the current action, moves the robot, calls the update belief service,
            gets the next action upon arriving at the next location, etc. It does not
            handle interrupts via a bump. It updates belief when not observing an obstacle.

            Parameters:
                msg     --  The Odometry message data.
                self    --  A reference to the relevant SigmaActionKobuki object.
        """

        rospy.loginfo("Info[SigmaActionKobuki]: Received odometry.")

        SigmaActionKobuki.check_reached_goal(msg, self)
        rospy.loginfo("Coolness")
        SigmaActionKobuki.move_to_goal(msg, self)
        rospy.loginfo("Awesome")

    @staticmethod
    def check_reached_goal(msg, self):
        """ Handle checking and reaching a goal.

            This means getting a new action from the SimgaPOMDP and setting variables,
            as well as doing distance calculations.

            Parameters:
                msg     --  The Odometry message data.
                self    --  A reference to the relevant SigmaActionKobuki object.
        """

        # Compute the distance to the goal given the positions.
        distanceToPositionGoal = math.sqrt(pow(self.x - self.goalX, 2) +
                                           pow(self.y - self.goalY, 2))

        rospy.loginfo("Step 0")

        # If the robot is far from the goal, with no bump detected either, then do nothing.
        if distanceToPositionGoal >= self.atPositionGoalThreshold and \
                not self.detectedBump:
            return

        rospy.loginfo("Step 1")

        # However, if it is close enough to the goal, then update the belief with
        # observing a bump or not.
        rospy.wait_for_service(self.srvUpdateBeliefTopic)
        srvUpdateBelief = rospy.ServiceProxy(self.srvUpdateBeliefTopic, UpdateBelief)
        res = srvUpdateBelief(self.goalX, self.goalY, self.detectedBump)
        if not res.success:
            rospy.logerror("Error[SigmaActionKobuki.check_reached_goal]: Failed to update belief.")
            return

        rospy.loginfo("Step 2")

        # The new 'origin' is the current pose estimates from the odometers.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Now do a service request for the SigmaPOMDP to send the current action.
        rospy.wait_for_service(self.srvGetActionTopic)
        srvGetAction = rospy.ServiceProxy(self.srvGetActionTopic, GetAction)
        res = srvGetAction()

        relGoalX = res.goal_x
        relGoalY = res.goal_y
        relGoalTheta = res.goal_theta

        rospy.loginfo("Step 3")

        # We need to translate the goal location given by srvGetAction to the world-frame.
        # They are provided as a relative goal. Theta, however, is given in 'world-frame'
        # kinda, basically because it is not in the SigmaPOMDP's state space.
        self.goalX = self.x + relGoalX
        self.goalY = self.y + relGoalY
        self.goalTheta = np.arctan2(self.goalY - self.y, self.goalX - self.x)

        # Finally, reset the bump detection because we have already incorporated that
        # in the belief update above.
        self.detectedBump = False

        rospy.loginfo("Step 4")

    @staticmethod
    def move_to_goal(msg, self):
        """ Move toward the goal using the relevant Kobuki messages.

            Parameters:
                msg     --  The Odometry message data.
                self    --  A reference to the relevant SigmaActionKobuki object.
        """

        rospy.loginfo("Step 5")

        #roll, pitch, self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        self.theta = 0.0

        control = Twist()
        control.linear.y = 0
        control.linear.z = 0
        control.angular.x = 0
        control.angular.y = 0

        # If close to the goal, then do nothing. Otherwise, drive based on normal control.
        distanceToPositionGoal = math.sqrt(pow(self.x - self.goalX, 2) +
                                           pow(self.y - self.goalY, 2))
        if distanceToPositionGoal < self.atPositionGoalThreshold:
            control.linear.x = 0.0
        else:
            # This assigns the desired set-point for speed in meters per second.
            control.linear.x = self.desiredVelocity #valP + valI + valD

        rospy.loginfo("Step 6")

        # Update the (noisy) position based on the odometers velocity estimates.
        self.x += msg.twist.twist.x
        self.y += msg.twist.twist.y

        # Compute the new goal theta based on the updated (noisy) location of the robot.
        # The result is in the range [-pi, pi], so we have to cast it to [0, 2pi].
        self.goalTheta = np.arctan2(self.goalY - self.y, self.goalX - self.x) + np.pi

        error = self.goalTheta - self.theta
        if abs(error) < self.atThetaGoalThreshold:
            control.angular.z = 0.0
        else:
            valP = error * self.thetaKp

            self.pidIntegrator += error
            self.pidIntegrator = np.clip(self.pidIntegrator,
                                         -self.pidIntegratorBounds,
                                         self.pidIntegratorBounds)
            valI = self.pidIntegrator * self.thetaKi

            self.pidDerivator = error - self.pidDerivator
            self.pidDerivator = error
            valD = self.pidDerivator * self.thetaKd

            # This assigns the desired set-point for relative angle.
            control.angular.z = valP + valI + valD

        rospy.loginfo("Step 7")

        self.pubKobukiVel.publish(control)

        rospy.loginfo("Step 8")

    @staticmethod
    def sub_kobuki_bump(msg, self):
        """ This method checks for sensing a bump.

            Parameters:
                msg     --  The BumperEvent message data.
                self    --  A reference to the relevant SigmaActionKobuki object.
        """

        self.detectedBump = (msg.state == BumperEvent.PRESSED)

