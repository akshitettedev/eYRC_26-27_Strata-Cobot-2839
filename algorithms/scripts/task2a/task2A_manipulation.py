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
# Filename:		    task2A_manipulation.py
# Functions:
#			        quat_to_z_axis, clamp_vector, clamp_scalar, segment_distance_to_origin,
#			        arm_manipulation.__init__, tcpposecb, jointstatecb, armstatuscb,
#			        forcecb, switch_controller, send_magnet, publish_twist,
#			        publish_jointjog, is_latched, warning_scale, set_phase, elapsed,
#			        move_to, start_travel, process, handle_* (one per phase), main
# Nodes:		    arm_manipulation_node
#			        Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status,
#                                         /ur7e/end_effector/force_magnitude ]
#                   Services used      - [ /magnet, /ur_arm_controller/switch_controller,
#                                         /ur_arm_controller/unlock_protective_stop ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32
from std_srvs.srv import Trigger, SetBool


##################### TASK CONSTANTS #######################

base_frame = 'base_link'

# The three kinds, exactly as they appear in the transform names from perception.
ore_kinds = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']
ORES_TOTAL = 4
MAX_ORES_PER_KIND = 4

# ---- FILL THESE IN: where each container is, in base_link, in metres. -------------
# (x, y) of the centre of each container's opening, and the tool height (tcp z) to drop
# from. Measure them once - see the guide. The node refuses to start while any is None.
# Hint: a held ore hangs 76 mm BELOW the tcp, so drop_tcp_z must be at least
# (container rim height + 0.076 + a few cm).
#
# While tuning you do NOT have to edit and rebuild: every number below and in the
# TUNING section can be overridden on the command line, e.g.
#   ros2 run algorithms task2A_manipulation.py --ros-args \
#       -p azurite_x:=-0.64 -p azurite_y:=0.68 -p drop_z:=0.15 -p force_contact:=0.3
# Once the values work, copy them in here so the evaluation run (which uses no
# overrides) uses them.
azurite_drop = None       # e.g. (-0.74, 0.53)
malachite_drop = None
vanadinite_drop = None
drop_tcp_z = None
# -----------------------------------------------------------------------------------

# The two command interfaces (only one is live at a time).
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# Input caps. A message over any of these is discarded whole, so stay under them.
cap_linear_mps = 0.15
cap_angular_rps = 0.35
cap_joint_rps = 0.30      # a little under the 0.35 cap


##################### TUNING CONSTANTS #######################

# Geometry of an ore (101.6 x 101.6 x 76.2 mm).
ORE_HALF_HEIGHT = 0.0381
ORE_HEIGHT = 0.0762

# Unfold out of the folded start pose (joint space), then switch to Cartesian.
unfold_joints = [0.0, -1.20, -1.60, -1.90, 1.57, 1.57]
unfold_tol = 0.03
Kp_joint = 1.0

# Task-space approach loop (same shape as Task 1B).
Kp_lin = 1.0
travel_speed = 0.08       # m/s for the big, free moves
descend_fast = 0.05       # m/s while still well above the ore
descend_slow = 0.015      # m/s for the last stretch before touching it (coasts a few mm)
slow_zone = 0.04          # m above the top face where the slow descent starts
Kp_ang = 0.4
ang_deadband = 0.02
warn_backoff = 0.25

# Heights, all measured from the ore's TOP face.
PREGRASP_CLEARANCE = 0.12   # tool this far above the ore before the final descent
PICK_LIFT = 0.10            # how far to lift straight up to check the hold
CARRY_CLEARANCE = 0.25      # carry height: the ore hangs 76 mm below the tool
DROP_CLEARANCE = 0.10       # carry height is also at least this far above drop_tcp_z
PRESS_DEPTH = 0.012         # aim this far below the top face; contact stops it sooner

# Tolerances.
tol_travel = 0.010
tol_pregrasp = 0.004
tol_drop = 0.008

# Contact and hold, from the force at the tool (tool weight already removed).
FORCE_CONTACT = 0.4         # N: the spring is pressing on the ore
CONTACT_TICKS = 2           # consecutive control cycles above that before stopping
HOLD_MIN = 0.8              # N: an ore (0.15 kg) reads about 1.47 N once lifted
HOLD_MAX = 2.2
HOLD_SETTLE_S = 0.8         # wait after lifting before reading the force
GRIP_DWELL_S = 0.6          # stay pressed on the ore with the magnet on before lifting
RELEASE_WAIT_S = 1.0        # wait after switching the magnet off so the ore falls clear

MAX_TRIES = 2               # pick attempts per ore
PHASE_TIMEOUT_S = 40.0      # give up on a phase that never finishes
MAX_RECOVERIES = 2          # protective stops allowed on one ore before it is skipped
REACH_WARN = 0.78           # m from the base axis: warn above this (UR7e reach is ~0.85 m)

# Latch recovery.
recovery_window_s = 5.0
retreat_speed = 0.03

# Phases that get a timeout.
TIMED_PHASES = ('TRAVEL_UP', 'TRAVEL_ACROSS', 'TRAVEL_DOWN', 'MAGNET_ON',
                'DESCEND', 'GRIP', 'LIFT', 'CHECK_HOLD', 'RELEASE')


##################### FUNCTION DEFINITIONS #######################

def quat_to_z_axis(x, y, z, w):
    '''
    Description:    The tool's own z-axis as a unit vector in base_link, from its quaternion.

    Args:
        x, y, z, w  (float):    Quaternion components

    Returns:
        axis        (tuple):    Unit vector
    '''

    vx = 2.0 * (x * z + y * w)
    vy = 2.0 * (y * z - x * w)
    vz = 1.0 - 2.0 * (x * x + y * y)
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    return (0.0, 0.0, 1.0) if n < 1e-9 else (vx / n, vy / n, vz / n)


def clamp_vector(vx, vy, vz, cap):
    '''
    Description:    Scale a 3-vector down so its magnitude does not exceed 'cap'.

    Args:
        vx, vy, vz  (float):    Components
        cap         (float):    Magnitude limit

    Returns:
        vx, vy, vz  (float):    Scaled components
    '''

    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    if n > cap and n > 1e-9:
        s = cap / n
        return vx * s, vy * s, vz * s
    return vx, vy, vz


def clamp_scalar(value, cap):
    '''
    Description:    Clamp one number to +/- cap.

    Args:
        value       (float):    Number to clamp
        cap         (float):    Limit

    Returns:
        value       (float):    Clamped number
    '''

    return max(-cap, min(cap, value))


def segment_distance_to_origin(ax, ay, bx, by):
    '''
    Description:    Shortest distance from the base axis (the origin of base_link, seen
                    from above) to the straight line between two points. A path that
                    passes close to the base drives the arm toward a singularity.

    Args:
        ax, ay      (float):    Start point
        bx, by      (float):    End point

    Returns:
        distance    (float):    Metres
    '''

    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-9:
        return math.sqrt(ax * ax + ay * ay)
    t = -(ax * dx + ay * dy) / length_sq
    t = max(0.0, min(1.0, t))
    px, py = ax + t * dx, ay + t * dy
    return math.sqrt(px * px + py * py)


##################### CLASS DEFINITION #######################

class arm_manipulation(Node):
    '''
    ___CLASS___

    Description:    Picks each ore off the conveyor with the magnet and drops it in the
                    container for its kind. One state machine, one phase per timer tick.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_manipulation
        '''

        Node.__init__(self, 'arm_manipulation_node',
            parameter_overrides=[Parameter(
                'use_sim_time', Parameter.Type.BOOL, True)])

        # Run with  --ros-args -p calibrate:=true  to unfold the arm and then just
        # print the tool pose, so the container positions can be measured by hand.
        self.declare_parameter('calibrate', False)
        self.calibrate = self.get_parameter('calibrate').value

        # Every tunable is a ROS parameter whose default is the constant at the top of
        # this file, so it can be changed per run without a rebuild.
        self.azurite_drop = self.drop_param('azurite', azurite_drop)
        self.malachite_drop = self.drop_param('malachite', malachite_drop)
        self.vanadinite_drop = self.drop_param('vanadinite', vanadinite_drop)
        self.drop_z = self.optional_param('drop_z', drop_tcp_z)
        self.force_contact = self.number_param('force_contact', FORCE_CONTACT)
        self.hold_min = self.number_param('hold_min', HOLD_MIN)
        self.hold_max = self.number_param('hold_max', HOLD_MAX)
        self.travel_speed = self.number_param('travel_speed', travel_speed)
        self.descend_fast = self.number_param('descend_fast', descend_fast)
        self.descend_slow = self.number_param('descend_slow', descend_slow)
        self.pregrasp_clearance = self.number_param('pregrasp_clearance', PREGRASP_CLEARANCE)
        self.press_depth = self.number_param('press_depth', PRESS_DEPTH)
        self.carry_clearance = self.number_param('carry_clearance', CARRY_CLEARANCE)
        self.release_wait = self.number_param('release_wait', RELEASE_WAIT_S)
        self.ore_height_offset = self.number_param('ore_half_height', ORE_HALF_HEIGHT)

        if not self.calibrate:
            missing = (self.azurite_drop is None or self.malachite_drop is None
                       or self.vanadinite_drop is None or self.drop_z is None)
            if missing:
                self.get_logger().fatal(
                    'Container positions are not set. Fill in azurite_drop, '
                    'malachite_drop, vanadinite_drop and drop_tcp_z at the top of '
                    'this file (run with -p calibrate:=true to measure them).')
                sys.exit(1)

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)
        self.force_sub = self.create_subscription(
            Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 20)

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05
        self.switch_cli = self.create_client(SwitchController, f'{servo_ns}/switch_controller')
        self.unlock_cli = self.create_client(Trigger, f'{servo_ns}/unlock_protective_stop')
        self.magnet_cli = self.create_client(SetBool, '/magnet')
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(control_rate, self.process)

        self.tcp_pose = None
        self.joint_angles = None
        self.arm_status = None
        self.force = 0.0

        # Phase machine.
        self.phase = 'UNFOLD' if self.calibrate else 'WAIT_ORES'
        self.phase_t0 = self.get_clock().now()
        self.switch_future = None
        self.magnet_future = None
        self.release_t0 = None
        self.after_release = None
        self.last_log_t = 0.0
        self.last_progress_t = 0.0

        # The orientation to hold (tool pointing down), captured after the unfold.
        self.desired_z = None

        # Ore bookkeeping.
        self.ore_names = []
        self.ore_index = 0
        self.tries = 0
        self.ore_x = 0.0
        self.ore_y = 0.0
        self.ore_top_z = 0.0
        self.drop_x = 0.0
        self.drop_y = 0.0
        self.transit_z = 0.0
        self.contact_ticks = 0
        self.picked_count = 0
        self.dropped_count = 0
        self.recoveries = 0           # protective stops cleared while on the current ore

        # Travel bookkeeping (up, across, down).
        self.travel_x = 0.0
        self.travel_y = 0.0
        self.travel_z = 0.0
        self.dest = (0.0, 0.0, 0.0)
        self.after_travel = None

        # Recovery bookkeeping.
        self.last_direction = (0.0, 0.0, 1.0)
        self.resume_phase = None
        self.unlock_future = None
        self.recover_t0 = None
        self.recover_stage = None

        if self.calibrate:
            # Calibrate mode starts the unfold at once, so go to joint servoing now.
            # Blocking is safe here: nothing is spinning yet.
            self.switch_controller(joint_controller, blocking=True)
        else:
            # The normal run waits for the ores first, then unfolds.
            self.get_logger().info('Waiting for four ore transforms from perception')


    def number_param(self, name, default):
        '''
        Description:    Declare one float ROS parameter whose default is a module constant.

        Args:
            name        (str):      Parameter name
            default     (float):    Value from the top of this file

        Returns:
            value       (float)
        '''

        self.declare_parameter(name, float(default))
        return self.get_parameter(name).get_parameter_value().double_value


    def optional_param(self, name, default):
        '''
        Description:    Declare one float ROS parameter that may be unset. None goes over the
                        wire as NaN and comes back as None.

        Args:
            name        (str):      Parameter name
            default     (float):    Value from the top of this file, or None if unset

        Returns:
            value       (float or None)
        '''

        self.declare_parameter(name, float('nan') if default is None else float(default))
        value = self.get_parameter(name).get_parameter_value().double_value
        return None if math.isnan(value) else value


    def drop_param(self, kind, default):
        '''
        Description:    Declare <kind>_x and <kind>_y parameters for one container.

        Args:
            kind        (str):      'azurite', 'malachite' or 'vanadinite'
            default     (tuple):    (x, y) from the top of this file, or None if unset

        Returns:
            drop        (tuple or None):  (x, y), or None unless both are set
        '''

        x = self.optional_param(f'{kind}_x', None if default is None else default[0])
        y = self.optional_param(f'{kind}_y', None if default is None else default[1])
        return None if x is None or y is None else (x, y)


    def tcpposecb(self, data):
        '''
        Description:    Keep the latest tool pose (base_link).

        Args:
            data (PoseStamped):    Tool pose
        '''

        self.tcp_pose = data.pose


    def jointstatecb(self, data):
        '''
        Description:    Keep the six joint angles, matched by name (the topic is alphabetical).

        Args:
            data (JointState):    Joint feedback
        '''

        if all(j in data.name for j in joint_names):
            self.joint_angles = [data.position[data.name.index(j)] for j in joint_names]


    def armstatuscb(self, data):
        '''
        Description:    Keep the arm's state code and log every change away from OK.

        Args:
            data (Int32):    State code
        '''

        previous = self.arm_status
        self.arm_status = data.data
        if self.arm_status not in [0, previous]:
            self.get_logger().warn(f'/arm_status changed to {self.arm_status}')


    def forcecb(self, data):
        '''
        Description:    Keep the force at the tool in newtons (tool weight already removed).

        Args:
            data (Float32):    Force magnitude
        '''

        self.force = data.data


    def switch_controller(self, controller, blocking=False):
        '''
        Description:    Make one of the two command interfaces the live one.

        Args:
            controller  (str):      'delta_twist_controller' or 'delta_joint_controller'
            blocking    (bool):     True waits here (only safe in __init__, before spin);
                                    False fires the request and the timer polls the result.

        Returns:
            success     (bool):     Whether the switch was confirmed (always True if not blocking)
        '''

        other = joint_controller if controller == twist_controller else twist_controller

        req = SwitchController.Request()
        req.activate_controllers = [controller]
        req.deactivate_controllers = [other]
        req.strictness = SwitchController.Request.STRICT

        self.switch_cli.wait_for_service(timeout_sec=20.0)

        if blocking:
            future = self.switch_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is not None:
                self.get_logger().info(f'Controller active: {controller}')
                return True
            self.get_logger().error(f'Failed to switch to {controller}')
            return False

        self.switch_future = self.switch_cli.call_async(req)
        self.pending_controller = controller
        return True


    def report_switch(self, result):
        '''
        Description:    Log what the servo said about a controller switch.

        Args:
            result      (SwitchController.Response or None)

        Returns:
            ok          (bool):     True if the servo accepted the switch
        '''

        if result is None:
            return False
        ok = bool(getattr(result, 'ok', True))
        message = getattr(result, 'message', '')
        self.get_logger().info(f'switch_controller reply: ok={ok} {message}')
        return ok


    def send_magnet(self, on):
        '''
        Description:    Switch the electromagnet on or off. The reply only confirms the call
                        arrived; whether an ore is held is read from the force instead.

        Args:
            on          (bool):     True energises the coil, False releases

        Returns:
            future      (Future):   The pending service call, polled by the caller
        '''

        self.magnet_cli.wait_for_service(timeout_sec=5.0)
        req = SetBool.Request()
        req.data = on
        return self.magnet_cli.call_async(req)


    def publish_twist(self, lx, ly, lz, ax, ay, az):
        '''
        Description:    Publish one TwistStamped on /delta_twist_cmds, stamped base_link.

        Args:
            lx, ly, lz  (float):    Linear velocity, m/s
            ax, ay, az  (float):    Angular velocity, rad/s
        '''

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.twist.linear.x = lx
        msg.twist.linear.y = ly
        msg.twist.linear.z = lz
        msg.twist.angular.x = ax
        msg.twist.angular.y = ay
        msg.twist.angular.z = az
        self.twist_pub.publish(msg)


    def publish_jointjog(self, velocities):
        '''
        Description:    Publish one JointJog naming all six joints.

        Args:
            velocities  (list):     One angular velocity per joint, rad/s
        '''

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(joint_names)
        msg.velocities = [clamp_scalar(v, cap_joint_rps) for v in velocities]
        self.joint_pub.publish(msg)


    def is_latched(self):
        '''
        Description:    True while /arm_status is a protective stop (codes 20 to 23).

        Returns:
            latched     (bool)
        '''

        return self.arm_status is not None and 20 <= self.arm_status < 24


    def warning_scale(self):
        '''
        Description:    Ease off while any WARNING_* (10 to 14) is active.

        Returns:
            scale       (float):    1.0 when healthy, warn_backoff during a warning
        '''

        if self.arm_status is not None and 10 <= self.arm_status < 20:
            return warn_backoff
        return 1.0


    def set_phase(self, name):
        '''
        Description:    Move the state machine to a new phase and restart its timeout clock.

        Args:
            name        (str):      Phase name
        '''

        self.phase = name
        self.phase_t0 = self.get_clock().now()
        self.get_logger().info(f'phase -> {name}')


    def elapsed(self, t0):
        '''
        Description:    Simulation seconds since t0.

        Args:
            t0          (Time):     Start time

        Returns:
            seconds     (float)
        '''

        return (self.get_clock().now() - t0).nanoseconds * 1e-9


    def hold_zero(self):
        '''
        Description:    Publish a zero twist so the arm holds still and its dead-man
                        timeout (0.15 s) keeps being fed.
        '''

        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


    def move_to(self, tx, ty, tz, cruise, tol):
        '''
        Description:    One control cycle of the approach loop: direction from the error,
                        speed = min(cruise, Kp * distance), tool held pointing down.

        Args:
            tx, ty, tz  (float):    Target tool position in base_link
            cruise      (float):    Top speed, m/s
            tol         (float):    Arrival distance, m

        Returns:
            arrived     (bool):     True once the tool is within 'tol' of the target
        '''

        if self.tcp_pose is None or self.desired_z is None:
            self.hold_zero()
            return False

        p = self.tcp_pose.position
        ex, ey, ez = tx - p.x, ty - p.y, tz - p.z
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)

        # Orientation: turn the tool's z-axis back onto the held direction.
        q = self.tcp_pose.orientation
        a = quat_to_z_axis(q.x, q.y, q.z, q.w)
        b = self.desired_z
        c = (a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0])
        c_norm = math.sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2])
        dot = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))
        angle = math.atan2(c_norm, dot) if c_norm > 1e-9 else 0.0

        if angle > ang_deadband:
            wx = Kp_ang * (c[0] / c_norm) * angle
            wy = Kp_ang * (c[1] / c_norm) * angle
            wz = Kp_ang * (c[2] / c_norm) * angle
        else:
            wx = wy = wz = 0.0
        wx, wy, wz = clamp_vector(wx, wy, wz, cap_angular_rps)

        scale = self.warning_scale()

        # Progress line every 2 s of simulation time: shows whether the tool is actually
        # closing on the target, and what the arm's safety state is while it does.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self.last_progress_t >= 2.0:
            self.last_progress_t = now_s
            self.get_logger().info(
                f'{self.phase}: tcp=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
                f'target=({tx:.3f}, {ty:.3f}, {tz:.3f}) dist={dist:.3f} m '
                f'status={self.arm_status} force={self.force:.2f} N')

        if dist < tol:
            self.publish_twist(0.0, 0.0, 0.0, wx * scale, wy * scale, wz * scale)
            return True

        direction = (ex / dist, ey / dist, ez / dist)
        speed = min(cruise, Kp_lin * dist)
        vx, vy, vz = clamp_vector(direction[0] * speed, direction[1] * speed,
                                  direction[2] * speed, cap_linear_mps)
        self.last_direction = direction
        self.publish_twist(vx * scale, vy * scale, vz * scale,
                           wx * scale, wy * scale, wz * scale)
        return False


    def start_travel(self, x, y, z, next_phase):
        '''
        Description:    Begin a safe three-leg move: straight up to the carry height, across
                        at that height, then straight down to (x, y, z). Then go to next_phase.

        Args:
            x, y, z     (float):    Destination, base_link
            next_phase  (str):      Phase to enter on arrival
        '''

        if self.tcp_pose is None:
            self.get_logger().warn('TCP pose unavailable; delaying travel until pose is received')
            self.hold_zero()
            return

        p = self.tcp_pose.position
        self.travel_x = p.x
        self.travel_y = p.y
        self.travel_z = max(self.transit_z, p.z)
        self.dest = (x, y, z)
        self.after_travel = next_phase
        self.set_phase('TRAVEL_UP')


    def process(self):  # sourcery skip: low-code-quality
        '''
        Description:    Timer function. Runs one tick of the phase machine. A latched
                        protective stop and a phase timeout are handled before anything else.
        '''

        if self.phase != 'RECOVERING' and self.is_latched():
            self._extracted_from_process_8()
        if self.phase in TIMED_PHASES and self.elapsed(self.phase_t0) > PHASE_TIMEOUT_S:
            return self._extracted_from_process_18()
        if self.phase == 'RECOVERING':
            self.handle_recovery()
        elif self.phase == 'WAIT_ORES':
            self.handle_wait_ores()
        elif self.phase == 'UNFOLD':
            self.handle_unfold()
        elif self.phase == 'SWITCHING':
            self.handle_switching()
        elif self.phase == 'CALIBRATE':
            self.handle_calibrate()
        elif self.phase == 'NEXT_ORE':
            self.handle_next_ore()
        elif self.phase == 'TRAVEL_UP':
            if self.move_to(self.travel_x, self.travel_y, self.travel_z, self.travel_speed, tol_travel):
                self.set_phase('TRAVEL_ACROSS')
        elif self.phase == 'TRAVEL_ACROSS':
            if self.move_to(self.dest[0], self.dest[1], self.travel_z, self.travel_speed, tol_travel):
                self.set_phase('TRAVEL_DOWN')
        elif self.phase == 'TRAVEL_DOWN':
            if self.move_to(self.dest[0], self.dest[1], self.dest[2], self.travel_speed, tol_pregrasp):
                self.set_phase(self.after_travel)
        elif self.phase == 'MAGNET_ON':
            self.handle_magnet_on()
        elif self.phase == 'DESCEND':
            self.handle_descend()
        elif self.phase == 'GRIP':
            self.hold_zero()
            if self.elapsed(self.phase_t0) >= GRIP_DWELL_S:
                self.set_phase('LIFT')
        elif self.phase == 'LIFT':
            self.handle_lift()
        elif self.phase == 'CHECK_HOLD':
            self.handle_check_hold()
        elif self.phase == 'RELEASE':
            self.handle_release()
        elif self.phase == 'PICK_FAILED':
            self.handle_pick_failed()
        elif self.phase == 'DROPPED':
            self.dropped_count += 1
            self.get_logger().info(f'Dropped {self.ore_names[self.ore_index]} '
                                   f'({self.dropped_count} of {len(self.ore_names)})')
            self.ore_index += 1
            self.set_phase('NEXT_ORE')
        else:
            # DONE: keep the dead-man switch fed.
            self.hold_zero()

    # TODO Rename this here and in `process`
    def _extracted_from_process_18(self):
        self.get_logger().error(
            f'{self.phase} timed out; giving up on this ore. arm_status={self.arm_status}, '
            f'force={self.force:.2f} N. Run: ros2 topic echo /arm_status_detail --once')
        self.send_magnet(False)
        self.ore_index += 1
        self.magnet_future = None
        self.set_phase('NEXT_ORE')
        return

    # TODO Rename this here and in `process`
    def _extracted_from_process_8(self):
        self.recoveries += 1
        self.get_logger().error(
            f'Latched: status {self.arm_status} (stop {self.recoveries} on this ore). Recovering.')
        self.resume_phase = self.phase
        self.phase = 'RECOVERING'
        self.recover_stage = 'unlocking'
        self.unlock_future = None
        self.recover_t0 = None


    def handle_wait_ores(self):
        '''
        Description:    Wait until perception has published all four ore transforms, then
                        remember their names and start unfolding.
        '''

        names = []
        for kind in ore_kinds:
            for i in range(1, MAX_ORES_PER_KIND + 1):
                name = f'{kind}_{i}'
                if self.tf_buffer.can_transform(base_frame, name, Time()):
                    names.append(name)

        if len(names) < ORES_TOTAL:
            return

        # Nearest to the base first: the ore at the edge of the reach goes last, so if it
        # latches the arm the other three are already in their containers.
        radius = {}
        for name in names:
            tf = self.tf_buffer.lookup_transform(base_frame, name, Time())
            radius[name] = math.hypot(tf.transform.translation.x, tf.transform.translation.y)
        names.sort(key=lambda n: radius[n])

        self.ore_names = names
        self.get_logger().info(f'Ores found (nearest first): {names}')
        self.switch_controller(joint_controller, blocking=False)
        self.set_phase('UNFOLD')


    def handle_unfold(self):
        '''
        Description:    Joint-servo from the folded start pose to an open pose, because a
                        Cartesian move from the start pose latches the arm at once.
        '''

        # Wait for the joint controller switch requested earlier before moving.
        if self.switch_future is not None:
            if not self.switch_future.done():
                self.publish_jointjog([0.0] * 6)
                return
            self.report_switch(self.switch_future.result())
            self.switch_future = None

        if self.joint_angles is None:
            self.publish_jointjog([0.0] * 6)
            return

        errors = [t - c for t, c in zip(unfold_joints, self.joint_angles)]

        if max(abs(e) for e in errors) < unfold_tol:
            self.publish_jointjog([0.0] * 6)
            self.get_logger().info('Unfold complete; switching to end-effector servoing')
            self.switch_controller(twist_controller, blocking=False)
            self.set_phase('SWITCHING')
            return

        self.publish_jointjog([clamp_scalar(Kp_joint * e, cap_joint_rps) for e in errors])


    def handle_switching(self):
        '''
        Description:    Wait for the switch back to end-effector servoing, capture the
                        tool orientation to hold, and check the tool points down.
        '''

        self.publish_jointjog([0.0] * 6)
        self.hold_zero()

        if self.switch_future is None or not self.switch_future.done():
            return
        result = self.switch_future.result()
        if result is None or not self.report_switch(result):
            self.get_logger().error('Switch to end-effector servoing was not accepted; retrying')
            self.switch_controller(twist_controller, blocking=False)
            return
        if self.tcp_pose is None:
            return

        q = self.tcp_pose.orientation
        self.desired_z = quat_to_z_axis(q.x, q.y, q.z, q.w)
        self.switch_future = None
        self.get_logger().info(
            f'Tool z-axis in base_link: ({self.desired_z[0]:.2f}, '
            f'{self.desired_z[1]:.2f}, {self.desired_z[2]:.2f})')

        if self.desired_z[2] > -0.95:
            self.get_logger().error(
                'The tool is NOT pointing straight down after the unfold. The magnet '
                'will not sit flat on the ore. Change unfold_joints (wrist_1 / wrist_2) '
                'until this line shows a z of about -1.00.')
            if not self.calibrate:
                self.set_phase('DONE')
                return

        if self.calibrate:
            self.set_phase('CALIBRATE')
        else:
            self.set_phase('NEXT_ORE')


    def handle_calibrate(self):
        '''
        Description:    Print the tool pose once a second and publish nothing, so the tool
                        can be jogged by hand from another terminal and read off.
        '''

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log_t < 1.0 or self.tcp_pose is None:
            return
        self.last_log_t = now
        p = self.tcp_pose.position
        self.get_logger().info(f'tcp in base_link: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')


    def handle_next_ore(self):
        '''
        Description:    Load the next ore's position from its transform, pick its container,
                        and start travelling to the pre-grasp point above it.
        '''

        if self.ore_index >= len(self.ore_names):
            self.get_logger().info(
                f'All ores handled: {self.dropped_count} of {len(self.ore_names)} dropped')
            self.set_phase('DONE')
            return

        if (self.azurite_drop is None or self.malachite_drop is None
                or self.vanadinite_drop is None or self.drop_z is None):
            self.get_logger().error('Container positions are not set')
            self.set_phase('DONE')
            return

        name = self.ore_names[self.ore_index]
        try:
            tf = self.tf_buffer.lookup_transform(base_frame, name, Time())
        except Exception as e:
            self.get_logger().warn(f'No transform for {name}: {e}')
            return

        self.ore_x = tf.transform.translation.x
        self.ore_y = tf.transform.translation.y
        self.ore_top_z = tf.transform.translation.z + self.ore_height_offset

        if name.startswith('azurite_ore'):
            self.drop_x, self.drop_y = self.azurite_drop
        elif name.startswith('malachite_ore'):
            self.drop_x, self.drop_y = self.malachite_drop
        else:
            self.drop_x, self.drop_y = self.vanadinite_drop

        self.transit_z = max(self.ore_top_z + self.carry_clearance, self.drop_z + DROP_CLEARANCE)
        self.tries = 0
        self.recoveries = 0

        near = segment_distance_to_origin(self.ore_x, self.ore_y, self.drop_x, self.drop_y)
        reach = math.hypot(self.ore_x, self.ore_y)
        self.get_logger().info(
            f'{name}: ore at ({self.ore_x:.3f}, {self.ore_y:.3f}), top face z={self.ore_top_z:.3f}, '
            f'{reach:.2f} m from the base; container at ({self.drop_x:.3f}, {self.drop_y:.3f})')
        if reach > REACH_WARN:
            self.get_logger().warn(
                f'{name} is {reach:.2f} m out: close to full stretch with the tool pointing '
                f'down. If the arm latches on a joint limit or singularity here, this is why.')
        if near < 0.30:
            self.get_logger().warn(
                f'The carry path passes {near:.2f} m from the base axis: watch for a singularity')

        self.start_travel(self.ore_x, self.ore_y,
                          self.ore_top_z + self.pregrasp_clearance, 'MAGNET_ON')


    def handle_magnet_on(self):
        '''
        Description:    Energise the magnet while hovering above the ore, wait for the
                        service reply, then start the descent.
        '''

        self.hold_zero()
        if self.magnet_future is None:
            self.magnet_future = self.send_magnet(True)
            return
        if not self.magnet_future.done():
            return
        reply = self.magnet_future.result()
        if reply is None:
            self.get_logger().error('Magnet service gave no reply; trying again')
            self.magnet_future = None
            return
        self.get_logger().info(f'Magnet ON reply: success={reply.success} {reply.message}')
        self.magnet_future = None
        self.contact_ticks = 0
        self.set_phase('DESCEND')


    def handle_descend(self):
        '''
        Description:    Straight down onto the ore's top face, fast then slow. Stops when
                        the force at the tool shows contact.
        '''

        if self.tcp_pose is None:
            self.hold_zero()
            return

        z = self.tcp_pose.position.z
        speed = self.descend_fast if z > self.ore_top_z + slow_zone else self.descend_slow
        arrived = self.move_to(self.ore_x, self.ore_y,
                               self.ore_top_z - self.press_depth, speed, tol_pregrasp)

        if self.force >= self.force_contact:
            self.contact_ticks += 1
        else:
            self.contact_ticks = 0

        if self.contact_ticks >= CONTACT_TICKS:
            p = self.tcp_pose.position
            self.get_logger().info(
                f'Contact at z={z:.3f} (force {self.force:.2f} N); tool is '
                f'{(p.x - self.ore_x) * 1000:+.1f} mm in x and {(p.y - self.ore_y) * 1000:+.1f} mm '
                f'in y from the ore centre')
            self.hold_zero()
            self.set_phase('GRIP')
        elif arrived:
            self.get_logger().warn('Reached full depth without a force reading; lifting anyway')
            self.set_phase('LIFT')


    def handle_lift(self):
        '''
        Description:    Straight up, slowly, with the ore on the magnet (if it took).
        '''

        if self.move_to(self.ore_x, self.ore_y, self.ore_top_z + PICK_LIFT,
                        self.descend_fast, tol_travel):
            self.set_phase('CHECK_HOLD')


    def handle_check_hold(self):
        '''
        Description:    Hover after the lift and read the force. An ore on the tool reads
                        about 1.47 N; free air reads about 0 N.
        '''

        self.hold_zero()
        if self.elapsed(self.phase_t0) < HOLD_SETTLE_S:
            return

        if self.hold_min <= self.force <= self.hold_max:
            self.picked_count += 1
            self.get_logger().info(f'Holding {self.ore_names[self.ore_index]} (force {self.force:.2f} N)')
            self.start_travel(self.drop_x, self.drop_y, self.drop_z, 'RELEASE')
            self.after_release = 'DROPPED'
        else:
            self.get_logger().warn(f'No ore on the tool (force {self.force:.2f} N)')
            self.after_release = 'PICK_FAILED'
            self.set_phase('RELEASE')


    def handle_release(self):
        '''
        Description:    Switch the magnet off, wait for the service reply, then wait a moment
                        for the ore to fall clear before moving on to after_release.
        '''

        self.hold_zero()
        if self.magnet_future is None:
            self.magnet_future = self.send_magnet(False)
            self.release_t0 = None
            return
        if not self.magnet_future.done():
            return
        if self.magnet_future.result() is None:
            self.magnet_future = None
            return
        if self.release_t0 is None:
            self.release_t0 = self.get_clock().now()
            return
        if self.elapsed(self.release_t0) < self.release_wait:
            return

        self.magnet_future = None
        self.release_t0 = None
        self.set_phase(self.after_release)


    def handle_pick_failed(self):
        '''
        Description:    The ore did not come up. Try again from the pre-grasp point, up to
                        MAX_TRIES times, then move on to the next ore.
        '''

        self.tries += 1
        if self.tries < MAX_TRIES:
            self.get_logger().warn(f'Retrying pick (attempt {self.tries + 1} of {MAX_TRIES})')
            self.start_travel(self.ore_x, self.ore_y,
                              self.ore_top_z + self.pregrasp_clearance, 'MAGNET_ON')
        else:
            self.get_logger().error(f'Giving up on {self.ore_names[self.ore_index]}')
            self.ore_index += 1
            self.set_phase('NEXT_ORE')


    def handle_recovery(self):
        '''
        Description:    Clear a latched protective stop: unlock, then back the tool away from
                        what it was driving into, inside the 5 s recovery window.
        '''

        if self.recover_stage == 'unlocking':
            if self.unlock_future is None:
                self.unlock_cli.wait_for_service(timeout_sec=5.0)
                self.unlock_future = self.unlock_cli.call_async(Trigger.Request())
            elif self.unlock_future.done():
                result = self.unlock_future.result()
                if result is not None and result.success:
                    self.get_logger().warn(f'Unlocked: {result.message}')
                    self.recover_stage = 'retreating'
                    self.recover_t0 = self.get_clock().now()
                else:
                    self.get_logger().error('unlock_protective_stop failed; retrying')
                    self.unlock_future = None
            self.hold_zero()
            return

        if self.arm_status == 0:
            self.hold_zero()
            if self.recoveries > MAX_RECOVERIES and self.resume_phase in TIMED_PHASES:
                return self._extracted_from_handle_recovery_28()
            self.get_logger().info('Recovered; resuming')
            return self._extracted_from_handle_recovery_35()
        if self.elapsed(self.recover_t0) > recovery_window_s - 0.5:
            self.get_logger().error('Recovery window closing without clearing; re-latch expected')
            self.hold_zero()
            return self._extracted_from_handle_recovery_35()
        d = self.last_direction
        self.publish_twist(-d[0] * retreat_speed, -d[1] * retreat_speed,
                           -d[2] * retreat_speed, 0.0, 0.0, 0.0)

    # TODO Rename this here and in `handle_recovery`
    def _extracted_from_handle_recovery_28(self):
        # The same ore keeps stopping the arm. Let it go and move on, so one
        # bad ore does not cost the other three.
        self.get_logger().error('Too many protective stops on this ore; skipping it')
        self.send_magnet(False)
        self.magnet_future = None
        self.ore_index += 1
        self.set_phase('NEXT_ORE')
        return

    # TODO Rename this here and in `handle_recovery`
    def _extracted_from_handle_recovery_35(self):
        self.phase = self.resume_phase
        self.phase_t0 = self.get_clock().now()
        return


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Create the manipulation node and spin it.
    '''

    rclpy.init(args=sys.argv)
    node = arm_manipulation()
    node.get_logger().info('Node created: Arm manipulation process')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()