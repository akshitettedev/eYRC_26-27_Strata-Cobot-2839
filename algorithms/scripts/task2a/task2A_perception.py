#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 2A of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task2A_perception.py
# Functions:
#			        mask_for, describe_contour, candidates_from, detect_ores,
#			        ore_tf.__init__, ore_tf.depthimagecb, ore_tf.colorimagecb,
#			        ore_tf.caminfocb, ore_tf.pixel_to_base, ore_tf.process_image,
#			        ore_tf.accumulate, ore_tf.broadcast_frozen, main
# Nodes:		    ore_tf_publisher
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                         /camera/camera/aligned_depth_to_color/image_raw,
#                                         /camera/camera/color/camera_info ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import tf2_ros
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point


##################### TASK CONSTANTS #######################

ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

base_frame = 'base_link'

# Task 2A puts exactly four ores on the conveyor. Unlike Task 1A the kinds are NOT
# two-of-each: the four can be any mix, so the per-type cap of 2 is gone.
ORES_TOTAL = 4


##################### TUNING CONSTANTS #######################

# Same colour bounds as Task 1A. (Vanadinite was the weak one in 1A - if an orange ore
# goes missing, run the hsv tuner on raw_frame.png and widen that entry first.)
hsv_bounds = {
    'azurite_ore':    [(np.array([ 90,  60,  50]), np.array([130, 255, 255]))],
    'malachite_ore':  [(np.array([ 35,  60,  50]), np.array([ 85, 255, 255]))],
    'vanadinite_ore': [(np.array([  3, 207, 209]), np.array([ 16, 255, 255]))],
}

RELAX_STEPS = [(0, 0), (30, 25), (55, 45)]

MIN_AREA = 150
MAX_AREA = 40000
MAX_RECT_ASPECT = 2.10
MIN_RECT_FILL = 0.60

# The ore is a 101.6 x 101.6 x 76.2 mm block. The camera sees its TOP face, and the ore
# is named by its CENTRE, so the published point is pushed down by half the height.
# (Task 1A used 0.015 here. The model's collision box is 76.2 mm tall, so 0.0381 is the
# centre. Check with tf2_echo against Gazebo before you trust either number.)
ORE_HALF_HEIGHT = 0.0381

# Lock-in. The arm is going to cross the camera's view, so the ores are measured once,
# early, while the view is clear, then the result is frozen and re-broadcast.
LOCK_FRAMES = 12        # this many consecutive consistent frames are averaged
MATCH_PIXELS = 15       # an ore must stay within this many pixels between frames


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

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def describe_contour(contour):
    '''
    Description:    Measure one contour and decide whether it looks like an ore face.

    Args:
        contour     (ndarray):  A single contour from cv2.findContours

    Returns:
        info        (dict):     area, center, bbox, ok (bool) and reason (str)
    '''

    area = cv2.contourArea(contour)
    info = {'area': area, 'center': None, 'bbox': None, 'ok': False, 'reason': ''}

    if area < MIN_AREA or area > MAX_AREA:
        info['reason'] = 'area out of range'
        return info

    info['bbox'] = cv2.boundingRect(contour)

    (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
    if rw <= 0 or rh <= 0:
        info['reason'] = 'degenerate rectangle'
        return info

    aspect = max(rw, rh) / min(rw, rh)
    fill = area / (rw * rh)

    M = cv2.moments(contour)
    if M['m00'] == 0:
        info['reason'] = 'zero moment'
        return info
    info['center'] = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

    if aspect > MAX_RECT_ASPECT or fill < MIN_RECT_FILL:
        info['reason'] = 'not square enough'
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
        found       (list):     (area, (cX, cY), (x, y, w, h)) per surviving blob
    '''

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for c in contours:
        info = describe_contour(c)
        if info['ok']:
            found.append((info['area'], info['center'], info['bbox']))
    return found


def detect_ores(image):
    '''
    Description:    Detect the ores in a colour frame. Every type is thresholded and
                    shape-filtered; the saturation/value floors are relaxed step by step
                    until at least ORES_TOTAL blobs are found, then the ORES_TOTAL
                    largest are kept.

    Args:
        image                   (ndarray):  BGR colour frame

    Returns:
        center_ore_list         (list):     (cX, cY) of every ore detected
        ore_type_list           (list):     Type of each ore, from 'ore_types'
        bbox_list               (list):     (x, y, w, h) of every ore detected
    '''

    center_ore_list = []
    ore_type_list = []
    bbox_list = []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    best = []
    for s_relax, v_relax in RELAX_STEPS:
        found = []
        for ore_type in ore_types:
            mask = mask_for(hsv, hsv_bounds[ore_type], s_relax, v_relax)
            found.extend(
                (area, center, bbox, ore_type)
                for area, center, bbox in candidates_from(mask)
            )
        if len(found) > len(best):
            best = found
        if len(found) >= ORES_TOTAL:
            break

    best.sort(key=lambda f: f[0], reverse=True)
    for area, center, bbox, ore_type in best[:ORES_TOTAL]:
        center_ore_list.append(center)
        ore_type_list.append(ore_type)
        bbox_list.append(bbox)

    return center_ore_list, ore_type_list, bbox_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Detects the four ores, averages their positions over a few clear
                    frames, then keeps broadcasting the frozen result as one tf2
                    transform per ore (kind_ore_N, in base_link).
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''

        Node.__init__(self, 'ore_tf_publisher',
            parameter_overrides=[Parameter(
                'use_sim_time', Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(image_processing_rate, self.process_image)

        self.cv_image = None
        self.depth_image = None
        self.cam_info = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.color_frame_id = None

        # Averaging state, all keyed by ore name.
        self.ref_pixel = {}       # pixel seen in the first frame of the current run
        self.sums = {}            # running sum of [x, y, z] in base_link
        self.frame_count = 0      # consistent frames collected so far

        # Once locked, this holds the final (x, y, z) per ore and is simply re-broadcast.
        self.locked = False
        self.frozen = {}
        self.last_annotated = None   # the frame shown in the window, kept after lock

        self.get_logger().info('Perception ready: waiting for a clear view of four ores')


    def depthimagecb(self, data):
        '''
        Description:    Store the aligned depth image, in metres.

        Args:
            data (Image):    Aligned depth frame (32FC1 metres, or 16UC1 millimetres)
        '''

        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)
        self.depth_image = depth


    def colorimagecb(self, data):
        '''
        Description:    Store the colour image as BGR.

        Args:
            data (Image):    Colour frame
        '''

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.color_frame_id = data.header.frame_id
        except CvBridgeError as e:
            self.get_logger().error(str(e))


    def caminfocb(self, data):
        '''
        Description:    Read the intrinsics at run time, never hardcoded.

        Args:
            data (CameraInfo):    Camera calibration
        '''

        k = data.k
        self.fx = k[0]
        self.cx = k[2]
        self.fy = k[4]
        self.cy = k[5]
        self.cam_info = data


    def pixel_to_base(self, cX, cY):
        '''
        Description:    Turn one colour pixel into a point in base_link: median depth of a
                        9x9 patch, back-projection with the intrinsics, then tf2.

        Args:
            cX, cY      (int):  Pixel column and row

        Returns:
            point       (tuple or None):  (x, y, z) in base_link, or None if no valid depth
                                          or no transform yet
        '''

        if self.depth_image is None:
            return None

        x0 = max(0, cX - 4)
        x1 = min(self.depth_image.shape[1], cX + 5)
        y0 = max(0, cY - 4)
        y1 = min(self.depth_image.shape[0], cY + 5)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))

        pt = PointStamped()
        if self.color_frame_id is None:
            self.get_logger().warn('Color frame ID not set yet')
            return None
        pt.header.frame_id = self.color_frame_id
        pt.point.x = float((cX - self.cx) * z / self.fx)
        pt.point.y = float((cY - self.cy) * z / self.fy)
        pt.point.z = z

        try:
            tf = self.tf_buffer.lookup_transform(base_frame, self.color_frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')
            return None

        p = do_transform_point(pt, tf)
        return (float(p.point.x), float(p.point.y), float(p.point.z) - ORE_HALF_HEIGHT)


    def accumulate(self, readings):
        '''
        Description:    Add one frame of readings to the running average. If this frame
                        does not match the previous ones (a different set of names, or an
                        ore that jumped more than MATCH_PIXELS), the average starts over
                        from this frame. After LOCK_FRAMES consistent frames the averages
                        are frozen.

        Args:
            readings    (list):  (name, cX, cY, x, y, z) for each of the four ores
        '''

        consistent = len(self.ref_pixel) == len(readings)
        if consistent:
            for name, cX, cY, x, y, z in readings:
                if name not in self.ref_pixel:
                    consistent = False
                    break
                rx, ry = self.ref_pixel[name]
                if abs(cX - rx) > MATCH_PIXELS or abs(cY - ry) > MATCH_PIXELS:
                    consistent = False
                    break

        if not consistent:
            self.ref_pixel = {}
            self.sums = {}
            self.frame_count = 0
            for name, cX, cY, x, y, z in readings:
                self.ref_pixel[name] = (cX, cY)
                self.sums[name] = [0.0, 0.0, 0.0]

        for name, cX, cY, x, y, z in readings:
            self.sums[name][0] += x
            self.sums[name][1] += y
            self.sums[name][2] += z
        self.frame_count += 1

        if self.frame_count >= LOCK_FRAMES:
            for name in self.sums:
                s = self.sums[name]
                self.frozen[name] = (s[0] / self.frame_count,
                                     s[1] / self.frame_count,
                                     s[2] / self.frame_count)
            self.locked = True
            for name in sorted(self.frozen):
                x, y, z = self.frozen[name]
                self.get_logger().info(f'LOCKED {name}: x={x:.3f} y={y:.3f} z={z:.3f} (base_link)')


    def broadcast_frozen(self):
        '''
        Description:    Re-broadcast every frozen ore transform with a fresh stamp. The
                        arm may hide the ores from the camera later; the transforms keep
                        coming regardless.
        '''

        stamp = self.get_clock().now().to_msg()
        transforms = []
        for name in self.frozen:
            x, y, z = self.frozen[name]
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = base_frame
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.br.sendTransform(transforms)


    def process_image(self):
        '''
        Description:    Timer function. Before lock: detect, name, measure and average.
                        After lock: only re-broadcast.
        '''

        if self.locked:
            self.broadcast_frozen()
            # Keep servicing the OpenCV window. If waitKey stops being called the
            # desktop marks the node as "not responding" and offers to kill it.
            if self.last_annotated is not None:
                cv2.imshow('detections', self.last_annotated)
            cv2.waitKey(1)
            return

        if (self.cv_image is None or self.depth_image is None
                or self.cam_info is None or self.color_frame_id is None):
            return

        centers, types, bboxes = detect_ores(self.cv_image)

        if len(centers) != ORES_TOTAL:
            self.frame_count = 0
            self.ref_pixel = {}
            self.get_logger().warn(f'Found {len(centers)} of {ORES_TOTAL} ores; not locking')
            return

        # Name each ore: kind first, then a number from 1 within that kind. Within a
        # kind the order is left-to-right, then top-to-bottom, as in Task 1A.
        readings = []
        annotated = self.cv_image.copy()
        for ore_type in ore_types:
            same = [(c, b) for c, t, b in zip(centers, types, bboxes) if t == ore_type]
            same.sort(key=lambda d: (d[0][0], d[0][1]))
            for idx, ((cX, cY), bbox) in enumerate(same):
                name = f'{ore_type}_{idx + 1}'
                point = self.pixel_to_base(cX, cY)
                if point is None:
                    return
                readings.append((name, cX, cY, point[0], point[1], point[2]))

                bx, by, bw, bh = bbox
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                cv2.putText(annotated, name, (bx, max(15, by - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        self.accumulate(readings)

        self.last_annotated = annotated
        cv2.imshow('detections', annotated)
        cv2.waitKey(1)

        if self.locked:
            cv2.imwrite('SC#2839_task2A_detection.png', annotated)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Create the perception node and spin it.
    '''

    rclpy.init(args=sys.argv)
    node = ore_tf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()