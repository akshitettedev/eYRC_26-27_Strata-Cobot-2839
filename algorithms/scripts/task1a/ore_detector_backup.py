#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          eYRC#2839
# Author List:		Akshit, Prapti, Ananya, Yash
# Filename:		    ore_detector.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point


##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

# The parent frame every ore transform is published against.
base_frame = 'base_link'


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
        bbox_list               (list):     Bounding box (x, y, w, h) of every ore detected
    '''

    ############ Function VARIABLES ############

    center_ore_list = []
    ore_type_list = []
    bbox_list = []

    ############ ADD YOUR CODE HERE ############

    # HSV colour bounds per ore type. H is 0-179 in OpenCV.
    # --- RELAXED VALUES FOR LIVE CAMERA ---
    hsv_bounds = {
        'azurite_ore':    [(np.array([ 90,  40,  40]), np.array([130, 255, 255]))],
        'malachite_ore':  [(np.array([ 35,  40,  40]), np.array([ 85, 255, 255]))],
        'vanadinite_ore': [(np.array([  0, 100, 100]), np.array([ 25, 255, 255]))],
    }
    min_area = 300

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for ore_type in ore_types:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in hsv_bounds[ore_type]:
            mask |= cv2.inRange(hsv, lo, hi)

        # Clean the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < min_area:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            
            # Get the bounding box for the ore
            x, y, w, h = cv2.boundingRect(c)
            bbox_list.append((x, y, w, h))
            
            center_ore_list.append((cX, cY))
            ore_type_list.append(ore_type)

    ############################################

    return center_ore_list, ore_type_list, bbox_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores in the cell and
                    broadcast a transform for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''

        super().__init__('ore_tf_publisher')                                            # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.cam_info = None                                                            # camera intrinsics variable (from caminfocb())

        ############ ADD YOUR CODE HERE ############

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.color_frame_id = None
        self.depth_frame_id = None

        # Keeps last-seen pixel of every named ore, so ids stay stable across frames.
        self.ore_memory = {}

        # Half the ore's height in metres. Measure in Gazebo and adjust if needed.
        self.ORE_HALF_HEIGHT = 0.015

        ############################################


    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
                        Use this function to receive the depth image and convert it to a CV2 image.

        Args:
            data (Image):    Input depth image frame received from the aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        # 16UC1 is millimetres, 32FC1 is already metres
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)

        self.depth_image = depth
        self.depth_frame_id = data.header.frame_id

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
                        Use this function to receive the raw image and convert it to a CV2 image.

        Args:
            data (Image):    Input coloured raw image frame received from the image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.color_frame_id = data.header.frame_id
        except CvBridgeError as e:
            self.get_logger().error(str(e))

        ############################################


    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.
                        Use this function to receive the camera's intrinsic parameters.

        Args:
            data (CameraInfo):    Camera calibration published by the camera

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        k = data.k
        self.fx = k[0]
        self.cx = k[2]
        self.fy = k[4]
        self.cy = k[5]
        self.cam_info = data

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect the ores and publish a transform for
                        each one on its estimated position.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        if self.cv_image is None or self.depth_image is None or self.cam_info is None:
            return

        centers, types, bboxes = detect_ores(self.cv_image)
        annotated = self.cv_image.copy()

        # ---- Group detections by type ----
        detections_by_type = {t: [] for t in ore_types}
        for (cX, cY), t, bbox in zip(centers, types, bboxes):
            detections_by_type[t].append(((cX, cY), bbox))

        # ---- Match detections to stable ids ----
        assignments = []              # (name, cX, cY, bbox)
        new_memory = {}
        for ore_type in ore_types:
            used_ids = set()
            for ((cX, cY), bbox) in detections_by_type[ore_type]:
                best_id, best_dist = None, 1e9
                for oid in (1, 2):
                    name = f'{ore_type}_{oid}'
                    if name in used_ids:
                        continue
                    if name in self.ore_memory:
                        px, py = self.ore_memory[name]
                        d = (px - cX) ** 2 + (py - cY) ** 2
                        if d < best_dist:
                            best_dist, best_id = d, oid
                if best_id is None:
                    for oid in (1, 2):
                        if oid not in used_ids:
                            best_id = oid
                            break
                if best_id is None:
                    continue
                used_ids.add(best_id)
                name = f'{ore_type}_{best_id}'
                new_memory[name] = (cX, cY)
                assignments.append((name, cX, cY, bbox))
        self.ore_memory = new_memory

        # ---- Publish a transform for every ore ----
        for name, cX, cY, bbox in assignments:
            # median over a small window
            x0 = max(0, cX - 4); x1 = min(self.depth_image.shape[1], cX + 5)
            y0 = max(0, cY - 4); y1 = min(self.depth_image.shape[0], cY + 5)
            patch = self.depth_image[y0:y1, x0:x1]
            valid = patch[np.isfinite(patch) & (patch > 0.0)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))

            # back-project pixel to camera optical frame
            X = (cX - self.cx) * z / self.fx
            Y = (cY - self.cy) * z / self.fy

            pt = PointStamped()
            pt.header.frame_id = self.color_frame_id
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = float(X)
            pt.point.y = float(Y)
            pt.point.z = float(z)

            source_frame = self.color_frame_id
            if source_frame is None:
                self.get_logger().warn('Colour camera frame is not available yet')
                continue

            try:
                tf = self.tf_buffer.lookup_transform(
                    base_frame, source_frame, Time())
            except Exception as e:
                self.get_logger().warn(f'TF not ready: {e}')
                continue

            p = do_transform_point(pt, tf)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = base_frame
            t.child_frame_id = name
            t.transform.translation.x = float(p.point.x)
            t.transform.translation.y = float(p.point.y)
            t.transform.translation.z = float(p.point.z) - self.ORE_HALF_HEIGHT
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)

            # ---- Annotate ----
            bx, by, bw, bh = bbox # Unpack the bounding box
            # Draw the boundary
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
            # Draw the center dot
            cv2.circle(annotated, (cX, cY), 6, (0, 255, 255), -1)
            
            # Draw a black background box for the text so it's always readable
            text_size, _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            text_w, text_h = text_size
            cv2.rectangle(annotated, (bx, by - text_h - 10), (bx + text_w, by), (0, 0, 0), -1)
            # Put the text on the black box
            cv2.putText(annotated, name, (bx, by - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow('detections', annotated)
        cv2.waitKey(1)
        
        # Save the annotated image with your team ID
        cv2.imwrite('SC#2839_task1A_detection.png', annotated)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the ore_tf
                    class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ore_tf_process')                      # creating ROS node

    node.get_logger().info('Node created: Ore tf process')          # logging information

    ore_tf_class = ore_tf()                                         # creating a new object for class 'ore_tf'

    rclpy.spin(ore_tf_class)                                        # spining on the object to make it alive in ROS 2 DDS

    ore_tf_class.destroy_node()                                     # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()