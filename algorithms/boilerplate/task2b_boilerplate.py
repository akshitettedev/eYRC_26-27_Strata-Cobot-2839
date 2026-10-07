#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 2B of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task2b_boilerplate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /ebot_path ]
#                   Subscribing Topics - [ /map, /odom ]
#                   Service Clients    - [ /spawn_ore_package ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_srvs.srv import Trigger


##################### TASK CONSTANTS #######################

# The map is published ONCE, as the world loads, and held for late subscribers.
map_topic = '/map'

# Your follower reads the path from here, with the same held-message QoS.
path_topic = '/ebot_path'

odom_topic = '/odom'
spawn_service = '/spawn_ore_package'

# Every pose in the path, and the path itself, is in this frame.
map_frame = 'map'

# The three points of the lap: (x, y) in metres and yaw in radians, in the map frame.
# At the ore drop pose and the arm pose either yaw, 0.0 or 3.14, is valid.
home_pose = (0.0, 0.0, 0.0)
ore_drop_pose = (6.0, 1.9, 0.0)
arm_pose = (0.0, 1.9, 0.0)

# The lap, as (start, end) of each step, in the order they are driven.
steps = [
    (home_pose, ore_drop_pose),     # step 1: then request the ore package
    (ore_drop_pose, arm_pose),      # step 2
    (arm_pose, home_pose),          # step 3
]


##################### CLASS DEFINITION #######################

class ebot_planner(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to read the map, plan a path for each
                    step of the lap, publish it on /ebot_path, and request the ore package
                    at the end of step 1.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_planner
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_planner_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # A held message: kept by the publisher and handed to anyone who subscribes later.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        ############ Topic SUBSCRIPTIONS ############

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.mapcb, latched)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)

        ############ Topic PUBLISHERS ############

        self.path_pub = self.create_publisher(Path, path_topic, latched)                # your follower drives what you publish here

        ############ Service CLIENTS ############

        self.spawn_client = self.create_client(Trigger, spawn_service)                  # drops the ore package onto the eBot

        ############ Constructor VARIABLES/OBJECTS ############

        plan_rate = 0.1                                                                 # rate of time to run one planning cycle (seconds)
        self.timer = self.create_timer(plan_rate, self.process_planning)                # creating a timer based function which gets called on every 0.1 seconds (as defined by 'plan_rate' variable)

        self.grid = None                                                                # the map (from mapcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.step = 0                                                                   # which step of the lap is next (index into 'steps')

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your planner needs to keep between cycles.
        #       ->  HINT: Whether the current step's path has been published, and whether
        #                 the ore package has been requested, for a start.

        ############################################


    def mapcb(self, data):
        '''
        Description:    Callback function for the map topic.
                        Use this function to receive the occupancy grid of the arena.

        Args:
            data (OccupancyGrid):    The arena, one value per cell

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the grid. Read the message definition first
        #       -> https://docs.ros2.org/latest/api/nav_msgs/msg/OccupancyGrid.html

        #   ->  The cells come as ONE flat list, row by row, starting at the map's origin-
        #           value = data.data[row * width + col]
        #       ->  HINT: data.info.width, data.info.height, data.info.resolution and
        #                 data.info.origin.position say how the grid sits in the map frame.

        #   ->  Every cell is free (0) or occupied (100). There are no unknown cells.

        #   ->  Log the size and the resolution once, so you can check them against RViz.

        ############################################


    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the position, the heading and the velocity of the base.
        #       ->  HINT: data.pose.pose.position, data.pose.pose.orientation (a quaternion)
        #                     yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        #                 data.twist.twist says whether it is standing still.

        ############################################


    def plan_path(self, start, end):
        '''
        Description:    Plans a path from 'start' to 'end' that stays clear of every rock.

        Args:
            start (tuple):    (x, y, yaw) where the step begins, in the map frame
            end (tuple):      (x, y, yaw) where the step ends, in the map frame

        Returns:
            list:             [(x, y), ...] from 'start' to 'end', or None if no path was found
        '''

        points = None

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Convert between map coordinates and grid cells, in both directions-
        #           col = int((x - origin_x) / resolution)
        #           x   = origin_x + (col + 0.5) * resolution      # the centre of the cell
        #       ->  NOTE: Most "the path crosses a rock" surprises are an off-by-one here.

        #   ->  Grow the rocks into a costmap first. The eBot is not a point: a path that
        #       only just misses a rock takes the chassis through it.
        #       ->  HINT: How much to grow them is the trade on the task page: too little and
        #                 the eBot clips a rock, too much and the path gets longer.

        #   ->  Search the costmap from the start cell to the end cell with any algorithm you
        #       write yourself (breadth-first, Dijkstra, A*, a sampling planner, ...).
        #       ->  NOTE: Ready-made planning or navigation packages are NOT allowed.

        #   ->  Turn the cells back into points, and thin them out. One point per cell is a
        #       jagged path that is longer than it needs to be.

        #   ->  The first and last points must be EXACTLY the step's start and end positions.

        ############################################

        return points


    def publish_path(self, points, start, end):
        '''
        Description:    Publishes 'points' on /ebot_path as a nav_msgs/Path.

        Args:
            points (list):    [(x, y), ...] as returned by plan_path()
            start (tuple):    (x, y, yaw) where the step begins
            end (tuple):      (x, y, yaw) where the step ends

        Returns:
        '''

        path = Path()
        path.header.frame_id = map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add one PoseStamped per point, each with header.frame_id = map_frame-
        #           pose = PoseStamped()
        #           pose.pose.position.x, pose.pose.position.y = x, y

        #   ->  Give the first and last poses the step's start and end yaw, as a quaternion-
        #           z = sin(yaw / 2), w = cos(yaw / 2)
        #       ->  HINT: The poses in between can face along the path.

        #   ->  Publish with self.path_pub. The last path published before the eBot moves
        #       off is the one that counts.

        ############################################


    def request_ore_package(self):
        '''
        Description:    Calls /spawn_ore_package once the eBot stands still on the ore drop pose.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Check the service is there first-
        #           self.spawn_client.service_is_ready()

        #   ->  Call it without blocking the node-
        #           future = self.spawn_client.call_async(Trigger.Request())
        #           future.add_done_callback(...)
        #       ->  NOTE: Never wait for the result inside a callback or a timer. The reply
        #                 is delivered by the same executor you would be blocking.

        #   ->  Log the reply's 'success' and 'message'. A call made while the eBot is still
        #       moving, or away from the ore drop pose, returns success: false.

        ############################################


    def process_planning(self):
        '''
        Description:    Timer function used to plan, publish and request, step by step.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Return early until the map and odometry have both arrived.

        #   ->  For the next step: plan with plan_path(), then publish_path(), while the eBot
        #       stands still at the step's start.

        #   ->  Watch /odom to know when your follower has finished the step: standing still
        #       within 0.3 m and 0.15 rad of the step's end pose.
        #       ->  HINT: At the end of step 1, request_ore_package() before moving on.

        #   ->  Then move on to the next step. Stop after step 3.

        #   ->  Nothing here should raise. An uncaught exception kills the node.

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_planner class to perform its task
    '''

    rclpy.init(args=sys.argv)                                           # initialisation

    node = rclpy.create_node('ebot_planner_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot planner process')        # logging information

    ebot_planner_class = ebot_planner()                                 # creating a new object for class 'ebot_planner'

    rclpy.spin(ebot_planner_class)                                      # spining on the object to make it alive in ROS 2 DDS

    ebot_planner_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                    # shutdown process


if __name__ == '__main__':

    main()
