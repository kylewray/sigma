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
from nova.pomdp import POMDP

import rospy
from nav_msgs.msg import OccupancyGrid

import itertools as it


SIGMA_OCCUPANCY_GRID_OBSTACLE_THRESHOLD = 50


class SigmaNode(object):
    """ The code which controls the robot following a POMDP using the sigma approximation. """

    def __init__(self):
        """ The constructor for the SigmaNode class. """

        # The POMDP and other related information.
        self.pomdp = POMDP()

        self.initialBeliefIsSet = False
        self.goalIsSet = False

        self.bestCaseProbabilityOfActionSuccess = \
                rospy.get_param("/sigma_node/best_case_probability_of_action_success", 0.8)

        # Information about the map for use by a path follower once our paths are published.
        self.mapWidth = 0
        self.mapHeight = 0
        self.mapOriginX = 0.0
        self.mapOriginY = 0.0
        self.mapResolution = 1.0

        # This is the number of x and y states that will be created using the map. Obstacle states
        # will, of course, be omitted.
        self.gridWidth = rospy.get_param("/sigma_node/grid_width", 10)
        self.gridHeight = rospy.get_param("/sigma_node/grid_height", 10)

        # Subscribers, publishers, services, etc. for ROS messages.
        self.subOccupancyGrid = None

    def initialize(self):
        """ Initialize the SigmaNode class, mainly registering subscribers and services. """

        subOccupancyGridTopic = rospy.get_param("/sigma_node/sub_occupancy_grid", "/map")
        self.subOccupancyGrid = rospy.Subscriber(subOccupancyGridTopic,
                                                OccupancyGrid,
                                                SigmaNode.sub_occupancy_grid,
                                                self)

    def update(self):
        """ Update the SigmaNode object. """

        # We only update once we have a valid POMDP.
        if self.pomdp is None or not self.initialBeliefIsSet or not self.goalIsSet:
            return

        #rospy.loginfo("Update function called.")
        pass

    @staticmethod
    def sub_occupancy_grid(msg, self):
        """ A subscriber handler for OccupancyGrid messages. This converges any 2d map
            into a set of POMDP states. This is a static method to work as a ROS callback.

            Parameters:
                msg     --  The OccupancyGrid message data.
                self    --  A reference to the relevant SigmaNode object.
        """

        rospy.logwarn("Map has been received. Creating a new POMDP.")

        # Remember map information.
        self.mapWidth = msg.info.width
        self.mapHeight = msg.info.height
        self.mapOriginX = msg.info.origin.position.x
        self.mapOriginY = msg.info.origin.position.y
        self.mapResolution = msg.info.resolution

        xStep = int(self.mapWidth / self.gridWidth)
        yStep = int(self.mapHeight / self.gridHeight)
        stateObstaclePercentage = dict()

        rospy.logwarn("%.2f %.2f %.2f %.2f %.4f" % (self.mapWidth, self.mapHeight, self.mapOriginX, self.mapOriginY, self.mapResolution))

        # Create a new POMDP every time the map is updated. This 'resets' the goal of course.
        self.pomdp = POMDP()
        self.initialBeliefIsSet = False
        self.goalIsSet = False

        self.pomdp.states = list(it.product(range(self.gridWidth), range(self.gridHeight)))
        for x, y in self.pomdp.states:
            stateObstaclePercentage[(x, y)] = 0

            pixelOffsets = list(it.product(range(xStep, yStep)))
            for i, j in pixelOffsets:
                if msg.data[(y + j) * self.mapWidth + (x + i)] >= SIGMA_OCCUPANCY_GRID_OBSTACLE_THRESHOLD:
                    stateObstaclePercentage[(x, y)] += 1

            stateObstaclePercentage[(x, y)] /= len(pixelOffsets)

        self.pomdp.n = len(self.pomdp.states)

        self.pomdp.actions = list(it.product([-1, 0, 1], [-1, 0, 1]))
        self.pomdp.m = len(self.pomdp.actions)

        self.pomdp.observations = list(['n/a', 'hit'])
        self.pomdp.z = len(self.pomdp.observations)

        # The number of successors is always only uncertain about left/right, or self-looping at center.
        self.ns = 4

        S = [[[int(-1) for sp in range(self.ns)] for a in range(self.m)] for s in range(self.n)]
        T = [[[float(0.0) for sp in range(self.ns)] for a in range(self.m)] for s in range(self.n)]
        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                cur = 0

                for sp, statePrime in enumerate(self.states):
                    # Begin with the pure probability of success and failure for each action.
                    prSuccess = 0.0
                    prFailDriftLeft = 0.0
                    prFailDriftRight = 0.0

                    # Now adjust these probabilities based on the obstacle percentage at that state.
                    prFailNoMove = 0.0

                    # TODO: Compute these probabilities, then create the state transitions by
                    # copy-pasting the code below a few times and filling in the correct values.

                    # TODO: Once this POMDP is made, you need two services: setgoal and setinitialpose
                    # These will listen for Rviz clicks, basically, but need to have parameters for the topic
                    # name. After setting each, be sure to set the variables for isGoalSet and isSetInitialBelief.
                    # Also, the initial pose assignment gives a collapsed initial belief. This function must
                    # additionally perform belief expansion!!

                    # TODO: Finally, the update method needs to run a few steps of Perseus (or PBVI). You
                    # should consider implementing GPU Perseus soon...

                    # TODO: Lastly, you also need a service called "computeAction" which returns a pose (maybe?)
                    # for the desired next cell location.
                    
                    # TODO: Once this is made, you'll need to write a simple
                    # path follower that basically calls computeAction, then nav_core path plans to this as a
                    # goal point, then once it arrives there calls computeAction again, etc.

                    #if True:
                    #    S[s][a][cur] = sp
                    #    T[s][a][cur] = 1.0
                    #    cur += 1

# Subscription to an OccupancyGrid. Converts any 2d map into a set of states.

# Service to a SetGoals message. Assigns floating-point goal states in the 2d map; converts to POMDP states.
# This really just creates the rewards.

# Method to update the pomdp solver by a fixed number of steps.

# Service to get the current best action at a particular state.

# Service to reset the belief to a collapsed state.


# Parameters:
# - grid_resolution_x -- how refined to make the grid. number is how many pixels to group together
# - grid_resolution_y -- how refined to make the grid. number is how many pixels to group together

#

