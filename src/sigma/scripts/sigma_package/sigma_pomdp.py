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
import nova.nova_pomdp as npm
import nova.pomdp_alpha_vectors as pav

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

from sigma.srv import *

import math
import itertools as it
import ctypes as ct
import numpy as np


SIGMA_OCCUPANCY_GRID_OBSTACLE_THRESHOLD = 50
SIGMA_GOAL_THRESHOLD = 0.001


class SigmaPOMDP(object):
    """ The code which controls the robot following a POMDP using the sigma approximation. """

    def __init__(self):
        """ The constructor for the SigmaPOMDP class. """

        # The POMDP and other related information.
        self.pomdp = POMDP()
        self.policy = ct.POINTER(pav.POMDPAlphaVectors)()
        self.belief = None

        self.initialBeliefIsSet = False
        self.goalIsSet = False
        self.algorithmIsInitialized = False

        self.stateObstaclePercentage = None

        self.baseProbabilityOfActionSuccess = rospy.get_param("~base_probability_of_action_success", 0.8)
        self.penaltyForFreespace = rospy.get_param("~penalty_for_freespace", 0.1)
        self.numberOfBeliefsToAdd = rospy.get_param("~number_of_beliefs_to_add", 10000)

        # Information about the map for use by a path follower once our paths are published.
        self.mapWidth = 0
        self.mapHeight = 0
        self.mapOriginX = 0.0
        self.mapOriginY = 0.0
        self.mapResolution = 1.0

        # This is the number of x and y states that will be created using the map. Obstacle states
        # will, of course, be omitted.
        self.gridWidth = rospy.get_param("~grid_width", 10)
        self.gridHeight = rospy.get_param("~grid_height", 10)

        # Subscribers, publishers, services, etc. for ROS messages.
        self.subOccupancyGrid = None
        self.subMapPoseEstimate = None
        self.subMapNavGoal = None

        self.srvGetAction = None
        self.srvGetBelief = None
        self.srvUpdateBelief = None

    def __del__(self):
        """ The deconstructor for the SigmaPOMDP class. """

        if self.algorithmIsInitialized:
            self.uninitializeAlgorithm()

    def initialize(self):
        """ Initialize the SigmaPOMDP class, mainly registering subscribers and services. """

        subOccupancyGridTopic = rospy.get_param("~sub_occupancy_grid", "/map")
        self.subOccupancyGrid = rospy.Subscriber(subOccupancyGridTopic,
                                                OccupancyGrid,
                                                 SigmaPOMDP.sub_occupancy_grid,
                                                self)

        subMapPoseEstimateTopic = rospy.get_param("~sub_map_pose_estimate", "/initialpose")
        self.subMapPoseEstimate = rospy.Subscriber(subMapPoseEstimateTopic,
                                                PoseWithCovarianceStamped,
                                                SigmaPOMDP.sub_map_pose_estimate,
                                                self)

        subMapNavGoalTopic = rospy.get_param("~sub_map_nav_goal", "/move_base_simple/goal")
        self.subMapNavGoal = rospy.Subscriber(subMapNavGoalTopic,
                                                PoseStamped,
                                                SigmaPOMDP.sub_map_nav_goal,
                                                self)

        srvGetActionTopic = rospy.get_param("~get_action", "get_action")
        self.srvGetAction = rospy.Service(srvGetActionTopic,
                                                GetAction,
                                                SigmaPOMDP.srv_get_action,
                                                self)

        srvGetBeliefTopic = rospy.get_param("~get_belief", "get_belief")
        self.srvGetBelief = rospy.Service(srvGetBeliefTopic,
                                                GetBelief,
                                                SigmaPOMDP.srv_get_belief,
                                                self)

        srvUpdateBeliefTopic = rospy.get_param("~update_belief", "update_belief")
        self.srvUpdateBelief = rospy.Service(srvUpdateBeliefTopic,
                                                UpdateBelief,
                                                SigmaPOMDP.srv_update_belief,
                                                self)

    def update(self):
        """ Update the SigmaPOMDP object. """

        # We only update once we have a valid POMDP.
        if self.pomdp is None or not self.initialBeliefIsSet or not self.goalIsSet:
            return

        # If this is the first time the POMDP has been ready to be updated, then
        # initialize necessary variables.
        if not self.algorithmIsInitialized:
            self.initialize_algorithm()

        rospy.loginfo("Info[SigmaPOMDP.update]: Updating the policy.")

        # This can be NOVA_SUCCESS or NOVA_CONVERGED.
        result = npm._nova.pomdp_perseus_update_cpu(self.pomdp)

    def initialize_algorithm(self):
        """ Initialize the POMDP algorithm. """

        if self.algorithmIsInitialized:
            rospy.logwarn("Warn[SigmaPOMDP.initialize_algorithm]: Algorithm is already initialized.")
            return

        initialGamma = np.array([[float(self.pomdp.Rmin / (1.0 - self.pomdp.gamma)) \
                                        for s in range(self.pomdp.n)] \
                                    for i in range(self.pomdp.r)])

        array_type_rn_float = ct.c_float * (self.pomdp.r * self.pomdp.n)
        initialGamma = array_type_rn_float(*initialGamma.flatten())

        rospy.loginfo("Info[SigmaPOMDP.initialize_algorithm]: Initializing the algorithm.")

        result = npm._nova.pomdp_perseus_initialize_cpu(self.pomdp, initialGamma)
        if result != 0:
            rospy.logerror("Error[SigmaPOMDP.initialize_algorithm]: Failed to initialize algorithm.")
            return

        self.algorithmIsInitialized = True

    def uninitialize_algorithm(self):
        """ Uninitialize the POMDP algorithm. """

        if not self.algorithmIsInitialized:
            rospy.logwarn("Warn[SigmaPOMDP.uninitialize_algorithm]: Algorithm has not been initialized.")
            return

        rospy.loginfo("Info[SigmaPOMDP.uninitialize_algorithm]: Uninitializing the algorithm.")

        self.policy = None

        result = npm._nova.pomdp_perseus_uninitialize_cpu(self.pomdp)
        if result != 0:
            rospy.logwarn("Warn[SigmaPOMDP.uninitialize_algorithm]: Failed to uninitialize algorithm.")

        self.algorithmIsInitialized = False

    @staticmethod
    def map_to_world(mx, my, self):
        """ Convert map coordinates (integers) to world coordinates (offset, resolution, floats).

            Parameters:
                mx      --  The map x coordinate.
                my      --  The map y coordinate.
                self    --  A reference to the relevant SigmaPOMDP object.

            Returns:
                wx      --  The resultant world x coordinate. This is the center of the grid cell.
                wy      --  The resultant world y coordinate. This is the center of the grid cell.
        """

        wx = self.mapOriginX + mx * self.mapResolution
        wy = self.mapOriginY + my * self.mapResolution

        wx = wx * self.gridWidth + self.gridWidth / 2.0
        wy = wy * self.gridHeight + self.gridHeight / 2.0

        return wx, wy

    @staticmethod
    def world_to_map(wx, wy, self):
        """ Convert world coordinates (offset, resolution, floats) to map coordinates (integers).

            Parameters:
                wx      --  The world x coordinate.
                wy      --  The world y coordinate.
                self    --  A reference to the relevant SigmaPOMDP object.

            Returns:
                mx      --  The resultant map x coordinate.
                my      --  The resultant map y coordinate.

            Exceptions:
                The world coordinate is outside of the map.
        """

        if wx < self.mapOriginX or wy < self.mapOriginY or \
                wx >= self.mapOriginX + self.mapWidth * self.mapResolution or \
                wy >= self.mapOriginY + self.mapHeight * self.mapResolution:
            raise Exception()

        mx = (wx - self.mapOriginX) / self.mapResolution
        my = (wy - self.mapOriginY) / self.mapResolution

        xSize = self.mapWidth / self.gridWidth
        ySize = self.mapHeight / self.gridHeight

        mx = int(mx / xSize)
        my = int(my / ySize)

        return mx, my

    @staticmethod
    def sub_occupancy_grid(msg, self):
        """ A subscriber handler for OccupancyGrid messages. This converges any 2d map
            into a set of POMDP states. This is a static method to work as a ROS callback.

            Parameters:
                msg     --  The OccupancyGrid message data.
                self    --  A reference to the relevant SigmaPOMDP object.
        """

        rospy.loginfo("Info[SigmaPOMDP.sub_occupancy_grid]: Received map. Creating a new POMDP.")

        # Remember map information.
        self.mapWidth = msg.info.width
        self.mapHeight = msg.info.height
        self.mapOriginX = msg.info.origin.position.x
        self.mapOriginY = msg.info.origin.position.y
        self.mapResolution = msg.info.resolution

        xStep = int(self.mapWidth / self.gridWidth)
        yStep = int(self.mapHeight / self.gridHeight)

        # Create a new POMDP every time the map is updated. This 'resets' the goal of course.
        self.pomdp = POMDP()

        self.initialBeliefIsSet = False
        self.goalIsSet = False
        if self.algorithmIsInitialized:
            self.uninitialize_algorithm()

        self.pomdp.states = list(it.product(range(self.gridWidth), range(self.gridHeight)))
        self.pomdp.n = len(self.pomdp.states)

        self.pomdp.actions = list(it.product([-1, 0, 1], [-1, 0, 1]))
        self.pomdp.m = len(self.pomdp.actions)

        self.pomdp.observations = list([False, True])
        self.pomdp.z = len(self.pomdp.observations)

        # Compute the left and right actions for later use computing probability of drifting.
        driftActions = dict()
        for ax, ay in self.pomdp.actions:
            # Three cases: zero, diagonal, or axis-aligned.
            if ax == 0 and ay == 0:
                driftActions[(ax, ay)] = None
            elif ax != 0 and ay != 0:
                driftActions[(ax, ay)] = [(ax, 0), (0, ay)]
            else:
                if ax != 0:
                    driftActions[(ax, ay)] = [(ax, -1), (ax, 1)]
                elif ay != 0:
                    driftActions[(ax, ay)] = [(-1, ay), (1, ay)]

        # Compute the percentage of obstacles within each state.
        self.stateObstaclePercentage = dict()
        for x, y in self.pomdp.states:
            self.stateObstaclePercentage[(x, y)] = 0

            pixelOffsets = list(it.product(range(xStep), range(yStep)))
            for i, j in pixelOffsets:
                if msg.data[(y * yStep + j) * self.mapWidth + (x * xStep + i)] >= SIGMA_OCCUPANCY_GRID_OBSTACLE_THRESHOLD:
                    self.stateObstaclePercentage[(x, y)] += 1

            self.stateObstaclePercentage[(x, y)] = float(self.stateObstaclePercentage[(x, y)]) / float(len(pixelOffsets))

        # The number of successors is always only uncertain about left/right, or self-looping at center.
        self.pomdp.ns = 4

        # Compute the state transitions!
        S = [[[int(-1) for sp in range(self.pomdp.ns)] for a in range(self.pomdp.m)] for s in range(self.pomdp.n)]
        T = [[[float(0.0) for sp in range(self.pomdp.ns)] for a in range(self.pomdp.m)] for s in range(self.pomdp.n)]

        for s, state in enumerate(self.pomdp.states):
            for a, action in enumerate(self.pomdp.actions):
                # First off, if the action is (0, 0) (i.e., no movement), then it automatically succeeds.
                if action == (0, 0):
                    S[s][a][0] = s
                    T[s][a][0] = 1.0
                    continue

                # Begin with the pure probability of success and failure for each action.
                prSuccess = self.baseProbabilityOfActionSuccess
                prFailDrift = [(1.0 - self.baseProbabilityOfActionSuccess) / 2.0,
                               (1.0 - self.baseProbabilityOfActionSuccess) / 2.0]
                prFailNoMove = 0.0

                cur = 0

                # Now adjust the success probability based on the obstacle percentage at that state.
                try:
                    statePrime = (state[0] + action[0], state[1] + action[1])
                    sp = self.pomdp.states.index(statePrime)

                    S[s][a][cur] = sp
                    T[s][a][cur] = prSuccess * (1.0 - self.stateObstaclePercentage[statePrime])
                    cur += 1

                    prFailNoMove += prSuccess * self.stateObstaclePercentage[statePrime]
                except ValueError:
                    prFailNoMove += prSuccess
                    prSuccess = 0.0

                # Do the same for drifting side-to-side.
                for i in [0, 1]:
                    try:
                        statePrime = (state[0] + driftActions[action][i][0], state[1] + driftActions[action][i][1])
                        sp = self.pomdp.states.index(statePrime)

                        S[s][a][cur] = sp
                        T[s][a][cur] = prFailDrift[i] * (1.0 - self.stateObstaclePercentage[statePrime])
                        cur += 1

                        prFailNoMove += prFailDrift[i] * self.stateObstaclePercentage[statePrime]
                    except ValueError:
                        prFailNoMove += prFailDrift[i]
                        prFailDrift[i] = 0.0

                # Finally, if there was a non-zero value for the probability of failure, then assign this, too.
                if prFailNoMove > 0.0:
                    S[s][a][cur] = s
                    T[s][a][cur] = prFailNoMove
                    cur += 1

        array_type_nmns_int = ct.c_int * (self.pomdp.n * self.pomdp.m * self.pomdp.ns)
        array_type_nmns_float = ct.c_float * (self.pomdp.n * self.pomdp.m * self.pomdp.ns)

        self.pomdp.S = array_type_nmns_int(*np.array(S).flatten())
        self.pomdp.T = array_type_nmns_float(*np.array(T).flatten())

        # Compute the observation function!
        O = [[[0.0 for z in range(self.pomdp.z)] for sp in range(self.pomdp.n)] for a in range(self.pomdp.m)]

        for a, action in enumerate(self.pomdp.actions):
            for sp, statePrime in enumerate(self.pomdp.states):
                # The probability of observing a hit at a state is (assumed to be) equal to the percentage of obstacles in the state.
                O[a][sp][0] = 1.0 - self.stateObstaclePercentage[statePrime]
                O[a][sp][1] = self.stateObstaclePercentage[statePrime]

        array_type_mnz_float = ct.c_float * (self.pomdp.m * self.pomdp.n * self.pomdp.z)
        self.pomdp.O = array_type_mnz_float(*np.array(O).flatten())

        self.pomdp.k = 1
        self.pomdp.gamma = 0.99
        self.pomdp.horizon = self.gridWidth * self.gridHeight

        #print(np.array(S))
        #print(np.array(T))
        #print(np.array(O))

    @staticmethod
    def sub_map_pose_estimate(msg, self):
        """ A subscriber handler for PoseWithCovarianceStamped messages. This is when an initial
            pose is assigned, inducing an initial belief. This is a static method to work as a
            ROS callback.

            Parameters:
                msg     --  The PoseWithCovarianceStamped message data.
                self    --  A reference to the relevant SigmaPOMDP object.
        """

        if self.pomdp is None:
            rospy.logwarn("Warn[SigmaPOMDP.sub_map_pose_estimate]: POMDP has not yet been defined.")
            return

        try:
            gridPoseEstimateX, gridPoseEstimateY = SigmaPOMDP.world_to_map(msg.pose.pose.position.x, msg.pose.pose.position.y, self)
            sInitialBelief = self.pomdp.states.index((gridPoseEstimateX, gridPoseEstimateY))
        except Exception:
            rospy.logwarn("Warn[SigmaPOMDP.sub_map_pose_estimate]: Pose estimate position is outside of map bounds.")
            return

        rospy.loginfo("Info[SigmaPOMDP.sub_map_pose_estimate]: Received pose estimate. Assigning POMDP initial beliefs.")

        # The seed belief is simply a collapsed belief on this initial pose.
        Z = [[sInitialBelief]]
        B = [[1.0]]

        self.pomdp.r = len(B)
        self.pomdp.rz = 1

        array_type_rrz_int = ct.c_int * (self.pomdp.r * self.pomdp.rz)
        array_type_rrz_float = ct.c_float * (self.pomdp.r * self.pomdp.rz)

        self.pomdp.Z = array_type_rrz_int(*np.array(Z).flatten())
        self.pomdp.B = array_type_rrz_float(*np.array(B).flatten())

        self.pomdp.expand(method='random', numBeliefsToAdd=self.numberOfBeliefsToAdd)

        self.belief = np.array([float(s == sInitialBelief) for s in range(self.pomdp.n)])

        self.initialBeliefIsSet = True

    @staticmethod
    def sub_map_nav_goal(msg, self):
        """ A subscriber handler for PoseStamped messages. This is called when a goal is provided,
            assigning the rewards for the POMDP. This is a static method to work as a ROS callback.

            Parameters:
                msg     --  The OccupancyGrid message data.
                self    --  A reference to the relevant SigmaPOMDP object.
        """

        if self.pomdp is None:
            rospy.logwarn("Warn[SigmaPOMDP.sub_map_nav_goal]: POMDP has not yet been defined.")
            return

        try:
            gridGoalX, gridGoalY = SigmaPOMDP.world_to_map(msg.pose.position.x, msg.pose.position.y, self)
            sGoal = self.pomdp.states.index((gridGoalX, gridGoalY))
        except Exception:
            rospy.logwarn("Warn[SigmaPOMDP.sub_map_nav_goal]: Goal position is outside of map bounds.")
            return

        rospy.loginfo("Info[SigmaPOMDP.sub_map_nav_goal]: Received goal. Assigning POMDP rewards.")

        R = [[0.0 for a in range(self.pomdp.m)] for s in range(self.pomdp.n)]

        for s, state in enumerate(self.pomdp.states):
            for a, action in enumerate(self.pomdp.actions):
                if gridGoalX == state[0] and gridGoalY == state[1]:
                    R[s][a] = 0.0
                else:
                    R[s][a] = min(-self.penaltyForFreespace, -self.stateObstaclePercentage[state])

        self.pomdp.Rmax = np.array(R).max()
        self.pomdp.Rmin = np.array(R).min()

        array_type_nm_float = ct.c_float * (self.pomdp.n * self.pomdp.m)

        self.pomdp.R = array_type_nm_float(*np.array(R).flatten())

        self.goalIsSet = True

        #print(np.array(R))

    @staticmethod
    def srv_get_action(req, self):
        """ This service returns an action based on the current belief.

            Parameters:
                req     --  The service request as part of GetAction.
                self    --  A reference to the relevant SigmaPOMDP object.

            Returns:
                The service response as part of GetAction.
        """

        if self.policy is None or self.belief is None:
            return GetActionResponse(0.0, 0.0, 0.0)

        value, actionIndex = self.policy.value_and_action(self.belief)

        # The relative goal is simply the relative location based on the "grid-ize-ation"
        # and resolution of the map.
        goalX, goalY = self.pomdp.actions[a]

        xSize = self.mapWidth / self.gridWidth
        ySize = self.mapHeight / self.gridHeight

        goalX *= xSize * self.mapResolution
        goalY *= ySize * self.mapResolution

        # For the goal theta, we assume no collision at the next state, unless
        # this is impossible.
        observationIndex = self.pomdp.observations.index(False)

        # TODO: Finish this off by computing the theta at the next belief point.
        # Note that the observation may be impossible, so check for this.. maybe use
        # a try-catch or something.
        valuePrime, actionIndexPrime = self.pomdp.belief_update(self.belief, a, ...
        goalTheta = ...

        return GetActionResponse(goalX, goalY, 

    @staticmethod
    def srv_get_belief(req, self):
        """ This service returns the current belief.

            Parameters:
                req     --  The service request as part of GetBelief.
                self    --  A reference to the relevant SigmaPOMDP object.

            Returns:
                The service response as part of GetBelief.
        """

        return GetBeliefResponse(self.belief.tolist())

    @staticmethod
    def srv_update_belief(req, self):
        """ This service updates the belief based on a given action and observation.

            Parameters:
                req     --  The service request as part of UpdateBelief.
                self    --  A reference to the relevant SigmaPOMDP object.

            Returns:
                The service response as part of UpdateBelief.
        """

        actionX = np.sign(req.goal_delta_x) *
                            float(abs(req.goal_delta_x) > SIGMA_GOAL_THRESHOLD))
        actionY = np.sign(req.goal_delta_y) *
                            float(abs(req.goal_delta_y) > SIGMA_GOAL_THRESHOLD))

        actionIndex = self.pomdp.actions.index([actionX, actionY])
        observationIndex = self.pomdp.observations.index(req.bump_observed)

        try:
            self.belief = self.pomdp.belief_update(self.belief, actionIndex,
                                                   observationIndex, bp)
        except:
            rospy.logerror("Error[SigmaPOMDP.srv_belief_update]: Failed to update belief.")
            return UpdateBeliefResponse(False)

        return UpdateBeliefReponse(True)
