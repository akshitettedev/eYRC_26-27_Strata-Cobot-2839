#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task1b.py
# Functions:
#			        quat_to_z_axis, clamp_vector, clamp_scalar, main
# Nodes:		    arm_waypoints_node
#                   Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # 1
    (-0.8000, -0.0005, 0.3967),   # 2
    (-0.7430,  0.5280, 0.1967),   # 3
    (-0.4097,  0.5280, 0.1967),   # 4
    (-0.0763,  0.5280, 0.1967),   # 5
]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.15     # magnitude of a twist's linear part
cap_angular_rps = 0.35    # magnitude of its angular part
cap_joint_rps = 0.35      # per joint, on its own

# Dead-man switch: the arm stops this long after the last message it received.
command_timeout_s = 0.15

# An open pose, elbow mid-range and clear of the wrist singularity the start pose sits
# near. This is only an illustration in the manual, not a pose to reproduce exactly - the
# one property that matters is the elbow sitting away from both of its limits.
unfold_joints = [0.0, -1.20, -1.60, -1.90, 1.57, 1.57]
unfold_tol = 0.03         # rad per joint, "close enough" to call the unfold done
Kp_joint = 1.0            # joint-space proportional gain, rad/s per rad of error

# Task-space approach loop.
Kp_lin = 1.0
cruise_speed = 0.08       # m/s, comfortably under the 0.15 m/s cap
Kp_ang = 0.4              # gentler than 1.0 - paired with ang_deadband below so the
                          # controller isn't fighting a tiny residual error into a
                          # singular wrist configuration
ang_deadband = 0.02       # rad (~1.1 deg) - below this, stop correcting orientation
warn_backoff = 0.25       # scale applied to every published velocity while any
                          # WARNING_* check (10-14) is active, so the arm eases off
                          # before the condition can escalate to CRITICAL
reach_tol = 0.005         # 5 mm - inside this, start the hold timer
hold_duration_s = 2.0     # every waypoint must be held at least this long
waypoint_timeout_s = 45.0 # give up on a waypoint that never settles, rather than hang forever

# Latch recovery.
recovery_window_s = 5.0   # must match the arm's own window - see the manual
retreat_speed = 0.03      # m/s, backing away from whatever tripped the check


##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        Node.__init__(self,                                                             # registering node
            'arm_waypoints_node',
            parameter_overrides=[Parameter(
                'use_sim_time', Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)    # end-effector velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)        # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.switch_cli = self.create_client(                                           # client used to pick which command topic is live
            SwitchController, f'{servo_ns}/switch_controller')
        self.unlock_cli = self.create_client(                                           # client used to clear a latched protective stop
            Trigger, f'{servo_ns}/unlock_protective_stop')
        self.timer = self.create_timer(control_rate, self.process_waypoints)            # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.tcp_pose = None                                                            # tool pose variable (from tcpposecb())
        self.joint_angles = None                                                        # joint feedback variable (from jointstatecb())
        self.arm_status = None                                                          # arm state code variable (from armstatuscb())

        ############ ADD YOUR CODE HERE ############

        # The run moves through four phases in order: UNFOLD (joint servoing, out of the
        # folded start pose), SWITCHING (waiting for the controller swap to be confirmed),
        # APPROACH (end-effector servoing through 'waypoints'), DONE. RECOVERING is a
        # detour taken from APPROACH (or, in principle, from UNFOLD) whenever a critical
        # check latches the arm.
        self.phase = 'UNFOLD'
        self.switch_future = None            # pending switch_controller() call, polled from the timer
        self.pending_controller = None

        # UNFOLD / APPROACH bookkeeping.
        self.wp_index = 0                    # which waypoint (0-based) is currently being driven to
        self.wp_t0 = None                    # sim-time the current waypoint attempt started, for the timeout
        self.hold_t0 = None                  # sim-time the current hold started, or None while still approaching
        self.desired_z = None                # tool z-axis to hold through the approach, captured once unfolded
        self.goal_sequence = None            # transit-and-hold goals, built after unfold
        self.reached_count = 0               # real waypoints held so far (for logging)
        self.last_direction = (0.0, 0.0, 1.0)  # last unit direction commanded, for retreating on a latch

        # RECOVERING bookkeeping.
        self.resume_phase = None             # phase to return to once recovered
        self.unlock_future = None
        self.recover_t0 = None
        self.recover_stage = None            # 'unlocking' or 'retreating'

        # Kick off the controller switch to joint servoing so the unfold can start.
        # __init__ runs before rclpy.spin(), so blocking here is safe - see
        # 'switch_controller' below and the manual's note on where it may block.
        self.switch_controller(joint_controller, blocking=True)

        ############################################


    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.
                        Use this function to receive where the tool currently is.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # /tcp_pose_raw is published from forward kinematics regardless of which
        # controller is active, so this keeps updating through the unfold as well.
        self.tcp_pose = data.pose

        ############################################


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.
                        Use this function to receive the current angle of each joint.

        Args:
            data (JointState):    Joint feedback published by the arm

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # Match by name - the order /joint_states arrives in is alphabetical, not the
        # chain order the commands use. A joint silently left out here would be
        # treated as zero downstream, which reads as a large error, so only fill this
        # in once every name has actually been seen.
        if all(j in data.name for j in joint_names):
            self.joint_angles = [data.position[data.name.index(j)] for j in joint_names]

        ############################################


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.
                        Use this function to receive the arm's current state code.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        previous = self.arm_status
        self.arm_status = data.data

        if self.arm_status != 0 and self.arm_status != previous:
            self.get_logger().warn(f'/arm_status changed to {self.arm_status}')

        ############################################


    def switch_controller(self, controller, blocking=False):
        '''
        Description:    Function to make one of the arm's two command interfaces the active
                        one, so that commands published to it are acted on.

        Args:
            controller  (str):      Name of the controller to activate, either
                                    'twist_controller' or 'joint_controller'
            blocking    (bool):     True to wait for the reply here (only safe from
                                    __init__); False to fire the request and let the
                                    timer poll for the result instead.

        Returns:
            success     (bool):     Whether the controller was activated. Always True
                                    when blocking is False - the timer checks the real
                                    result once the request completes.
        '''

        ############ ADD YOUR CODE HERE ############

        other = joint_controller if controller == twist_controller else twist_controller

        req = SwitchController.Request()
        req.activate_controllers = [controller]
        req.deactivate_controllers = [other]
        req.strictness = SwitchController.Request.STRICT

        self.switch_cli.wait_for_service(timeout_sec=20.0)

        if blocking:
            # Only safe here because __init__ runs before rclpy.spin(self) starts -
            # nothing else is spinning yet to deliver the reply otherwise.
            future = self.switch_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.result() is not None:
                self.get_logger().info(f'Controller active: {controller}')
                return True
            self.get_logger().error(f'Failed to switch to {controller}')
            return False

        # From a timer callback, call() / spin_until_future_complete() would deadlock:
        # the executor that would deliver the reply is the same one blocked inside this
        # callback. Fire the request and let process_waypoints() poll it instead.
        self.switch_future = self.switch_cli.call_async(req)
        self.pending_controller = controller
        return True

        ############################################


    def quat_to_z_axis(self, x, y, z, w):
        '''
        Description:    Return the tool's own z-axis as a unit vector, from its quaternion.
                        This is the third column of the equivalent rotation matrix.

        Args:
            x   (float):    Quaternion x component
            y   (float):    Quaternion y component
            z   (float):    Quaternion z component
            w   (float):    Quaternion w component

        Returns:
            axis    (tuple[float, float, float]):  Unit vector, the tool's z-axis in base_link
        '''

        vx = 2.0 * (x * z + y * w)
        vy = 2.0 * (y * z - x * w)
        vz = 1.0 - 2.0 * (x * x + y * y)
        n = math.sqrt(vx * vx + vy * vy + vz * vz)
        if n < 1e-9:
            return (0.0, 0.0, 1.0)
        return (vx / n, vy / n, vz / n)


    def clamp_vector(self, vx, vy, vz, cap):
        '''
        Description:    Scale a 3-vector down so its magnitude does not exceed 'cap',
                        keeping its direction. Leaves it alone if it is already inside.

        Args:
            vx      (float):    X component
            vy      (float):    Y component
            vz      (float):    Z component
            cap     (float):    Magnitude limit

        Returns:
            vx      (float):    X component, scaled
            vy      (float):    Y component, scaled
            vz      (float):    Z component, scaled
        '''

        n = math.sqrt(vx * vx + vy * vy + vz * vz)
        if n > cap and n > 1e-9:
            s = cap / n
            return vx * s, vy * s, vz * s
        return vx, vy, vz


    def clamp_scalar(self, value, cap):
        '''
        Description:    Clamp a single number to +/- cap.

        Args:
            value   (float):    Number to clamp
            cap     (float):    Limit, applied symmetrically

        Returns:
            value   (float):    The clamped number
        '''

        if value > cap:
            return cap
        if value < -cap:
            return -cap
        return value


    def publish_twist(self, lx, ly, lz, ax, ay, az):
        '''
        Description:    Build and publish one TwistStamped on /delta_twist_cmds, stamped
                        in 'base_frame' as the servo requires.

        Args:
            lx      (float):    Linear velocity, x, m/s
            ly      (float):    Linear velocity, y, m/s
            lz      (float):    Linear velocity, z, m/s
            ax      (float):    Angular velocity, x, rad/s
            ay      (float):    Angular velocity, y, rad/s
            az      (float):    Angular velocity, z, rad/s

        Returns:
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
        Description:    Build and publish one JointJog on /delta_joint_cmds, naming every
                        joint in 'joint_names' so the message is never dropped for a
                        length mismatch.

        Args:
            velocities  (list[float]):  One angular velocity per entry of 'joint_names', rad/s

        Returns:
        '''

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(joint_names)
        msg.velocities = [self.clamp_scalar(v, cap_joint_rps) for v in velocities]
        self.joint_pub.publish(msg)


    def is_latched(self):
        '''
        Description:    Whether the last /arm_status code is a latched critical fault
                        (the CRITICAL_* codes, 20-23) that needs unlocking before
                        anything will move again.

        Args:
        Returns:
            latched     (bool):     True if the arm is currently latched
        '''

        return self.arm_status is not None and 20 <= self.arm_status < 24


    def warning_scale(self):
        '''
        Description:    Scale factor to apply to every published velocity while a
                        WARNING_* check (codes 10-14) is active, so the arm eases off
                        before the condition can escalate to CRITICAL instead of
                        continuing to drive into it.

        Args:
        Returns:
            scale       (float):     1.0 when healthy, 'warn_backoff' during a warning
        '''

        if self.arm_status is not None and 10 <= self.arm_status < 20:
            return warn_backoff
        return 1.0


    def process_waypoints(self):
        '''
        Description:    Timer function used to drive the tool through the waypoints. Runs
                        one of four phases each tick: unfolding out of the start pose,
                        waiting for the controller switch, the task-space approach to each
                        waypoint in turn, and recovering from a latched protective stop
                        should one occur along the way.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # A critical, latched check overrides whatever phase we were in, from anywhere
        # except a recovery already in progress.
        if self.phase != 'RECOVERING' and self.is_latched():
            self.get_logger().error(f'Latched: status {self.arm_status}. Recovering.')
            self.resume_phase = self.phase
            self.phase = 'RECOVERING'
            self.recover_stage = 'unlocking'
            self.unlock_future = None
            self.recover_t0 = None

        if self.phase == 'RECOVERING':
            self.handle_recovery()
            return

        if self.phase == 'UNFOLD':
            self.handle_unfold()
            return

        if self.phase == 'SWITCHING':
            self.handle_switching()
            return

        if self.phase == 'APPROACH':
            self.handle_approach()
            return

        # DONE: all five waypoints reached and held. Keep the dead-man switch fed.
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        ############################################


    def handle_unfold(self):
        '''
        Description:    Drive every joint, by joint servoing, from the folded start pose
                        toward 'unfold_joints'. The elbow starts 20 degrees from its limit
                        and a Cartesian move from there latches the arm almost at once, so
                        this has to happen in joint space before any waypoint is attempted.

        Args:
        Returns:
        '''

        if self.joint_angles is None:
            self.publish_jointjog([0.0] * 6)
            return

        errors = [t - c for t, c in zip(unfold_joints, self.joint_angles)]

        if max(abs(e) for e in errors) < unfold_tol:
            # Unfolded. Stop the joints, request the switch back to end-effector
            # servoing, and move on to waiting for it to take effect.
            self.publish_jointjog([0.0] * 6)
            self.get_logger().info('Unfold complete; switching to end-effector servoing')
            self.switch_controller(twist_controller, blocking=False)
            self.phase = 'SWITCHING'
            return

        velocities = [self.clamp_scalar(Kp_joint * e, cap_joint_rps) for e in errors]
        self.publish_jointjog(velocities)


    def handle_switching(self):
        '''
        Description:    Wait, without blocking the executor, for the switch_controller()
                        call requested at the end of the unfold to complete. Captures the
                        orientation to hold through the approach once it has.

        Args:
        Returns:
        '''

        # Keep both dead-man switches fed while we wait.
        self.publish_jointjog([0.0] * 6)
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        if self.switch_future is None or not self.switch_future.done():
            return

        if self.switch_future.result() is None:
            self.get_logger().error('switch_controller call failed; retrying')
            self.switch_controller(self.pending_controller, blocking=False)
            return

        self.get_logger().info(f'Controller active: {self.pending_controller}')

        # Hold whatever orientation the tool ended up at once unfolded, for the whole
        # approach - nothing in Task 1B asks for a different one at each waypoint.
        if self.tcp_pose is None:
            return

        q = self.tcp_pose.orientation
        self.desired_z = self.quat_to_z_axis(q.x, q.y, q.z, q.w)

        # Build a transit-and-hold goal sequence. W2 and W3 sit near maximum reach,
        # so the straight line from W3 to W4 passes the wrist through a singular
        # configuration. Between consecutive waypoints, insert a pull-in transit
        # goal: retract toward the base and cross at a moderate altitude, then
        # descend onto the next waypoint.
        PULL_IN = 0.55
        TRANSIT_Z = 0.40
        self.goal_sequence = []
        for i, (x, y, z) in enumerate(waypoints):
            if i > 0:
                px, py, _ = waypoints[i - 1]
                self.goal_sequence.append((px * PULL_IN, py * PULL_IN, TRANSIT_Z, False))
                self.goal_sequence.append((x  * PULL_IN, y  * PULL_IN, TRANSIT_Z, False))
            self.goal_sequence.append((x, y, z, True))

        self.wp_index = 0
        self.reached_count = 0
        self.wp_t0 = self.get_clock().now()
        self.hold_t0 = None
        self.phase = 'APPROACH'


    def handle_approach(self):
        '''
        Description:    The task-space approach loop. Drives the tool, by end-effector
                        servoing, straight at the current waypoint, holds it there for
                        'hold_duration_s' once reached, then moves on to the next one.

        Args:
        Returns:
        '''

        if self.tcp_pose is None or self.desired_z is None:
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        if self.goal_sequence is None or self.wp_index >= len(self.goal_sequence):
            self.phase = 'DONE'
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.get_logger().info('All waypoints reached and held.')
            return

        # --- Position error, current goal ---
        tx, ty, tz, need_hold = self.goal_sequence[self.wp_index]
        cx, cy, cz = self.tcp_pose.position.x, self.tcp_pose.position.y, self.tcp_pose.position.z
        ex, ey, ez = tx - cx, ty - cy, tz - cz
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)

        # --- Orientation error: turn the tool's current z-axis onto 'desired_z' ---
        q = self.tcp_pose.orientation
        a = self.quat_to_z_axis(q.x, q.y, q.z, q.w)
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
            # Residual orientation error is small enough to leave alone. Chasing it
            # to zero at some tool positions drives the wrist through a singularity -
            # this is what was causing the repeated CRITICAL_SINGULARITY latches.
            wx = wy = wz = 0.0
        wx, wy, wz = self.clamp_vector(wx, wy, wz, cap_angular_rps)

        # Ease off the moment a WARNING_* check (10-14) is active, rather than
        # continuing to press in and letting it escalate to a CRITICAL latch.
        scale = self.warning_scale()
        wx, wy, wz = wx * scale, wy * scale, wz * scale

        if dist < reach_tol:
            if not need_hold:
                # Transit goal: no hold, advance immediately.
                self.wp_index += 1
                self.wp_t0 = self.get_clock().now()
                self.hold_t0 = None
                self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                return
            # Real waypoint: hold zero linear velocity (orientation correction keeps
            # running) for 'hold_duration_s' of simulated time before moving on.
            if self.hold_t0 is None:
                self.hold_t0 = self.get_clock().now()
                self.reached_count += 1
                self.get_logger().info(
                    f'Waypoint {self.reached_count} reached; holding {hold_duration_s}s')
            elapsed = (self.get_clock().now() - self.hold_t0).nanoseconds * 1e-9
            if elapsed >= hold_duration_s:
                self.get_logger().info(
                    f'Waypoint {self.reached_count} held for {elapsed:.2f}s')
                self.wp_index += 1
                self.wp_t0 = self.get_clock().now()
                self.hold_t0 = None
                self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                return
            self.publish_twist(0.0, 0.0, 0.0, wx, wy, wz)
            return

        # Not yet reached: a hold in progress is invalidated by drifting back out.
        self.hold_t0 = None

        # A waypoint that never settles gives up rather than running out the clock.
        if self.wp_t0 is not None:
            running = (self.get_clock().now() - self.wp_t0).nanoseconds * 1e-9
            if running > waypoint_timeout_s:
                self.get_logger().error(
                    f'Waypoint {self.wp_index + 1} timed out after {running:.1f}s; skipping it')
                self.wp_index += 1
                self.wp_t0 = self.get_clock().now()
                self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                return

        # Direction and speed kept separate, per the approach loop in the manual: the
        # unit vector survives the cruise/taper choice, and the taper is one min().
        direction = (ex / dist, ey / dist, ez / dist)
        speed = min(cruise_speed, Kp_lin * dist)
        vx, vy, vz = direction[0] * speed, direction[1] * speed, direction[2] * speed
        vx, vy, vz = self.clamp_vector(vx, vy, vz, cap_linear_mps)
        vx, vy, vz = vx * scale, vy * scale, vz * scale

        self.last_direction = direction

        self.get_logger().debug(f'wp {self.wp_index + 1}: dist={dist:.4f} m, speed={speed:.3f} m/s')
        self.publish_twist(vx, vy, vz, wx, wy, wz)


    def handle_recovery(self):
        '''
        Description:    Clear a latched protective stop: unlock it, then back the tool
                        away from whatever it was driving into before resuming. Recovery
                        only proceeds while the 5 s window the manual describes is open.

        Args:
        Returns:
        '''

        if self.recover_stage == 'unlocking':
            if self.unlock_future is None:
                self.unlock_cli.wait_for_service(timeout_sec=5.0)
                self.unlock_future = self.unlock_cli.call_async(Trigger.Request())
                self.recover_t0 = self.get_clock().now()
            elif self.unlock_future.done():
                result = self.unlock_future.result()
                if result is not None and result.success:
                    self.get_logger().warn(f'Unlocked: {result.message}')
                else:
                    self.get_logger().error('unlock_protective_stop reported failure; retrying')
                    self.unlock_future = None
                    return
                self.recover_stage = 'retreating'
                self.recover_t0 = self.get_clock().now()
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        # 'retreating': stream a slow move opposite the direction we were last
        # commanding, which is only recovery motion accepted inside the window - and
        # stop as soon as either the fault clears or the window is about to run out.
        if self.recover_t0 is None:
            self.recover_t0 = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - self.recover_t0).nanoseconds * 1e-9
        if self.arm_status == 0:
            self.get_logger().info('Recovered; resuming')
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.phase = self.resume_phase
            if self.phase == 'APPROACH':
                self.wp_t0 = self.get_clock().now()
            return

        if elapsed > recovery_window_s - 0.5:
            self.get_logger().error('Recovery window closing without clearing; re-latching expected')
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.phase = self.resume_phase
            return

        dx, dy, dz = -self.last_direction[0], -self.last_direction[1], -self.last_direction[2]
        self.publish_twist(dx * retreat_speed, dy * retreat_speed, dz * retreat_speed, 0.0, 0.0, 0.0)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('arm_waypoints_process')               # creating ROS node

    node.get_logger().info('Node created: Arm waypoints process')   # logging information

    arm_waypoints_class = arm_waypoints()                           # creating a new object for class 'arm_waypoints'

    rclpy.spin(arm_waypoints_class)                                 # spining on the object to make it alive in ROS 2 DDS

    arm_waypoints_class.destroy_node()                              # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()