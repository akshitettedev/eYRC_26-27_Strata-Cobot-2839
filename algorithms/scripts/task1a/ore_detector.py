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
#			        mask_for, describe_contour, candidates_from, detect_ores,
#			        ore_tf.__init__, ore_tf.depthimagecb, ore_tf.colorimagecb,
#			        ore_tf.caminfocb, ore_tf.process_image, main
# Nodes:		    ore_tf_publisher
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                         /camera/camera/aligned_depth_to_color/image_raw,
#                                         /camera/camera/color/camera_info ]


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

# Exactly two ores of each type are spawned.
ORES_PER_TYPE = 2


##################### TUNING CONSTANTS #######################

# HSV colour bounds per ore type. H is 0-179 in OpenCV, S and V are 0-255.
# Each entry is a LIST of ranges, so a colour that straddles the 0/179 hue seam
# can be described with two. Vanadinite (orange) sits against that seam, so it
# gets a low-hue and a high-hue range; with only one, a deep orange-red face is
# split in half and each half falls under the area floor.
hsv_bounds = {
    'azurite_ore':    [(np.array([ 90,  60,  50]), np.array([130, 255, 255]))],
    'malachite_ore':  [(np.array([ 35,  60,  50]), np.array([ 85, 255, 255]))],
    'vanadinite_ore': [(np.array([  3, 207, 209]), np.array([ 16, 255, 255]))],
}

# Saturation and value floors are relaxed by these amounts, one step at a time,
# for any ore type that comes back with fewer than two detections. A shadowed
# face is the usual reason a first pass finds only one.
RELAX_STEPS = [(0, 0), (30, 25), (55, 45)]

# Shape gates. These are measured against a ROTATED rectangle fitted to the
# contour, not an axis-aligned bounding box.
#
#   Why: an ore's top face is a square, and a square rotated 45 degrees fills
#   only HALF of its axis-aligned bounding box. An axis-aligned fill test
#   therefore rejects every ore that is not neatly lined up with the image.
#   Fill against the fitted rectangle stays near 1.0 at any angle.
MIN_AREA = 150          # px, smallest blob kept
MAX_AREA = 40000        # px, largest blob kept - rejects a whole lit surface
MAX_RECT_ASPECT = 2.10  # long side / short side of the fitted rectangle
MIN_RECT_FILL = 0.60    # contour area / fitted rectangle area

# Set to True to have every rejected contour printed with its reason.
DEBUG_SHAPES = False


##################### FUNCTION DEFINITIONS #######################

def mask_for(hsv, ranges, s_relax=0, v_relax=0):
    '''
    Description:    Build one cleaned binary mask from a list of HSV ranges,
                    with the saturation and value floors optionally lowered.

    Args:
        hsv         (ndarray):  Frame already converted to HSV
        ranges      (list):     List of (lower, upper) ndarray pairs
        s_relax     (int):      Amount to lower the saturation floor by
        v_relax     (int):      Amount to lower the value floor by

    Returns:
        mask        (ndarray):  Cleaned binary mask
    '''

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        lo = lo.copy()
        lo[1] = max(0, int(lo[1]) - s_relax)
        lo[2] = max(0, int(lo[2]) - v_relax)
        mask |= cv2.inRange(hsv, lo, hi)

    # Open removes speckle, close fills the gaps a glare highlight leaves behind.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def describe_contour(contour):
    '''
    Description:    Measure one contour and decide whether it looks like an ore
                    face. Kept separate from the filtering so the debug script
                    can report the same numbers the detector acts on.

    Args:
        contour     (ndarray):  A single contour from cv2.findContours

    Returns:
        info        (dict):     area, center, bbox, rect_aspect, rect_fill,
                                ok (bool) and reason (str)
    '''

    info = {'area': 0.0, 'center': None, 'bbox': None,
            'rect_aspect': 0.0, 'rect_fill': 0.0,
            'ok': False, 'reason': ''}

    area = cv2.contourArea(contour)
    info['area'] = area

    if area < MIN_AREA:
        info['reason'] = f'area {area:.0f} < MIN_AREA {MIN_AREA}'
        return info
    if area > MAX_AREA:
        info['reason'] = f'area {area:.0f} > MAX_AREA {MAX_AREA}'
        return info

    info['bbox'] = cv2.boundingRect(contour)

    # Fit a rectangle that is free to rotate with the ore.
    (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
    if rw <= 0 or rh <= 0:
        info['reason'] = 'degenerate rectangle'
        return info

    long_side, short_side = max(rw, rh), min(rw, rh)
    info['rect_aspect'] = long_side / short_side
    info['rect_fill'] = area / (rw * rh)

    M = cv2.moments(contour)
    if M['m00'] == 0:
        info['reason'] = 'zero moment'
        return info
    info['center'] = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

    if info['rect_aspect'] > MAX_RECT_ASPECT:
        info['reason'] = (f"aspect {info['rect_aspect']:.2f} > "
                          f'MAX_RECT_ASPECT {MAX_RECT_ASPECT}')
        return info
    if info['rect_fill'] < MIN_RECT_FILL:
        info['reason'] = (f"fill {info['rect_fill']:.2f} < "
                          f'MIN_RECT_FILL {MIN_RECT_FILL}')
        return info

    info['ok'] = True
    info['reason'] = 'accepted'
    return info


def candidates_from(mask):
    '''
    Description:    Reduce a mask to the ore-shaped blobs in it.

    Args:
        mask        (ndarray):  Binary mask for one ore type

    Returns:
        found       (list):     (area, (cX, cY), (x, y, w, h)) per surviving
                                blob, largest first
    '''

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []

    for c in contours:
        info = describe_contour(c)
        if info['ok']:
            found.append((info['area'], info['center'], info['bbox']))
        elif DEBUG_SHAPES and info['area'] >= MIN_AREA:
            print(f"  rejected: {info['reason']}")

    found.sort(key=lambda f: f[0], reverse=True)
    return found


def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

                    Each type is thresholded on its own, filtered on shape, and
                    capped at the two largest surviving blobs. A type that comes
                    back short is retried with its saturation and value floors
                    relaxed, which recovers an ore face lying in shadow.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
        bbox_list gedit ~/ros2_ws/src/algorithms/scripts/task1a/ore_detector.py              (list):     Bounding box (x, y, w, h) of every ore detected
    '''

    ############ Function VARIABLES ############

    center_ore_list = []
    ore_type_list = []
    bbox_list = []

    ############ ADD YOUR CODE HERE ############

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for ore_type in ore_types:
        best = []
        for s_relax, v_relax in RELAX_STEPS:
            mask = mask_for(hsv, hsv_bounds[ore_type], s_relax, v_relax)
            best = candidates_from(mask)
            # Stop as soon as both ores of this type are accounted for. Relaxing
            # further would only start letting the background in.
            if len(best) >= ORES_PER_TYPE:
                break

        for _, center, bbox in best[:ORES_PER_TYPE]:
            center_ore_list.append(center)
            ore_type_list.append(ore_type)
            bbox_list.append(bbox)

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

        # Running mean of each ore's published position, which takes the few
        # millimetres of frame-to-frame depth noise out of the answer.
        self.position_filter = {}
        self.FILTER_ALPHA = 0.25

        # Half the ore's height in metres. The camera sees the top face; the ore
        # is named by its middle, so the published point is pushed down by this.
        self.ORE_HALF_HEIGHT = 0.015

        # One raw frame is written on start-up, for the tuner and debug scripts.
        self.saved_raw = False

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
            return

        # Drop one untouched frame on disk the first time through, so the tuner
        # and the debug script work on exactly what the node is seeing.
        if not self.saved_raw:
            cv2.imwrite('raw_frame.png', self.cv_image)
            self.get_logger().info('Wrote raw_frame.png')
            self.saved_raw = True

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
            dets = detections_by_type[ore_type]

            # With no history, name them by pixel position so the rule is the
            # same on every run: left-to-right, then top-to-bottom.
            if not any(f'{ore_type}_{i}' in self.ore_memory for i in (1, 2)):
                dets = sorted(dets, key=lambda d: (d[0][0], d[0][1]))
                for idx, ((cX, cY), bbox) in enumerate(dets[:ORES_PER_TYPE]):
                    name = f'{ore_type}_{idx + 1}'
                    new_memory[name] = (cX, cY)
                    assignments.append((name, cX, cY, bbox))
                continue

            used_ids = set()
            for ((cX, cY), bbox) in dets:
                best_id, best_dist = None, 1e9
                for oid in (1, 2):
                    name = f'{ore_type}_{oid}'
                    if oid in used_ids:
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

        # Carry forward any ore missed on this frame, so one bad frame does not
        # hand its id to the other ore of the same type on the next one.
        for name, px in self.ore_memory.items():
            new_memory.setdefault(name, px)
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

            source_frame = self.color_frame_id
            if source_frame is None:
                self.get_logger().warn('Colour camera frame is not available yet')
                continue

            pt = PointStamped()
            pt.header.frame_id = source_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = float(X)
            pt.point.y = float(Y)
            pt.point.z = float(z)

            try:
                tf = self.tf_buffer.lookup_transform(
                    base_frame, source_frame, Time())
            except Exception as e:
                self.get_logger().warn(f'TF not ready: {e}')
                continue

            p = do_transform_point(pt, tf)

            px = float(p.point.x)
            py = float(p.point.y)
            pz = float(p.point.z) - self.ORE_HALF_HEIGHT

            # Smooth the depth noise out over successive frames.
            if name in self.position_filter:
                ox, oy, oz = self.position_filter[name]
                a = self.FILTER_ALPHA
                px = ox + a * (px - ox)
                py = oy + a * (py - oy)
                pz = oz + a * (pz - oz)
            self.position_filter[name] = (px, py, pz)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = base_frame
            t.child_frame_id = name
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)

            # ---- Annotate ----
            bx, by, bw, bh = bbox
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
            cv2.circle(annotated, (cX, cY), 6, (0, 255, 255), -1)

            # Black plate behind the label, so it reads over any ore colour.
            text_size, _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            text_w, text_h = text_size
            ty = max(text_h + 10, by)
            cv2.rectangle(annotated, (bx, ty - text_h - 10), (bx + text_w, ty), (0, 0, 0), -1)
            cv2.putText(annotated, name, (bx, ty - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Say which types came up short, rather than only how many are missing.
        expected = len(ore_types) * ORES_PER_TYPE
        if len(assignments) < expected:
            short = [t for t in ore_types
                     if len(detections_by_type[t]) < ORES_PER_TYPE]
            self.get_logger().warn(
                f'Only {len(assignments)} of {expected} ores detected. '
                f"Short on: {', '.join(short) if short else 'id matching'}")

        cv2.imshow('detections', annotated)
        cv2.waitKey(1)

        # Only overwrite the submission image on a frame that has all six ores.
        if len(assignments) == expected:
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
