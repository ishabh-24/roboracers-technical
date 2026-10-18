#!/usr/bin/env python3

################################################################################
#
# wall_follow_node.py
#
# LiDAR-based wall follower for the AutoDRIVE RoboRacer simulator.
#
# The node subscribes to the 2D LiDAR scan, estimates the distance and heading
# relative to the track walls with the classic two-beam geometry, projects that
# distance a short lookahead ahead of the car, and runs a PID loop on the
# projected error to produce a steering command. As the free distance ahead
# shrinks, a follow-the-gap term (with disparity extension) is blended in so
# hairpins are turned into rather than driven into.
#
# Longitudinally the node computes a target speed from the stopping distance
# ahead, the lateral acceleration limit for the turn it is about to make and a
# "be slow before the turn-in" rule, then tracks it with a feed-forward plus
# proportional throttle law using the odometry speed as a speedometer. If the
# car ever ends up blocked against a wall it backs out and resumes.
#
# Three follow modes are available through the ``wall`` parameter:
#   * ``center``  keep the car centred between the left and right walls (default)
#   * ``left``    hold ``desired_distance`` metres from the left wall
#   * ``right``   hold ``desired_distance`` metres from the right wall
#
# Every gain is a ROS 2 parameter and can be changed at runtime with
# ``ros2 param set /wall_follow_node <name> <value>`` so the car can be tuned
# without rebuilding.
#
################################################################################

import math
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

################################################################################

# Steering command of 1.0 corresponds to roughly this steering angle (rad).
MAX_STEER_ANGLE = 0.52

# Default parameter values. Keys double as ROS 2 parameter names.
DEFAULTS = {
    # Which wall to follow: 'center', 'left' or 'right'.
    'wall': 'center',
    # Target distance to the followed wall (m). Only used for left/right modes.
    'desired_distance': 0.7,
    # Angle between the perpendicular beam and the forward-looking beam (deg).
    'theta_deg': 50.0,
    # Half-width of the beam window averaged for each measurement (beams).
    'beam_window': 3,
    # Projection distance ahead of the car used for the error term (m).
    'lookahead': 0.6,
    # Side ranges are clipped to this value so an open side does not dominate (m).
    'max_side_range': 2.5,
    # PID gains on the projected distance error -> steering angle (rad / m).
    'kp': 1.2,
    'ki': 0.0,
    'kd': 0.06,
    'integral_limit': 0.5,
    # Time constant of the low-pass filter on the derivative term (s).
    'derivative_tau': 0.06,
    # Low-pass factor on the steering command, 0 = none, 1 = frozen.
    'steer_smoothing': 0.15,
    # Gap following (steer towards the deepest free direction). It is always
    # blended in with ``gap_min_weight`` and takes over fully as the free
    # distance ahead drops below ``gap_full_dist``.
    'gap_min_weight': 0.35,
    'gap_full_dist': 2.5,
    'gap_sector_deg': 100.0,
    'gap_bubble_radius': 0.35,
    'gap_depth_fraction': 0.85,
    'gap_max_range': 6.0,
    # Range jump between neighbouring beams treated as a wall edge (m).
    'disparity_threshold': 0.3,
    # Half-angle of the cone used for the free distance ahead (deg).
    'front_cone_deg': 5.0,
    # Longitudinal model: steady-state speed is roughly this many m/s per unit
    # of throttle command (measured in the simulator).
    'speed_per_throttle': 25.0,
    # Speed envelope (m/s).
    'v_max': 10.0,
    'v_min': 1.2,
    # The car must be able to stop within the free distance ahead minus
    # ``stop_margin`` while decelerating at ``brake_decel`` (m, m/s^2).
    'brake_decel': 5.0,
    'stop_margin': 0.5,
    # It must also be down to ``turn_speed`` by the time the wall ahead is
    # ``turn_dist`` away, so braking is finished before the turn-in.
    'turn_speed': 2.0,
    'turn_dist': 1.6,
    # Lateral acceleration limit used to slow for the commanded steering
    # angle and for the turn needed to reach the gap (m/s^2); wheelbase (m).
    'lat_accel': 5.0,
    'wheelbase': 0.33,
    # The turn radius needed to reach the gap is computed pure-pursuit style
    # from the gap angle and this capped lookahead distance (m).
    'gap_lookahead': 2.0,
    # Closed-loop speed control. The measured speed comes from the odometry
    # topic when ``use_odom_speed`` is true, otherwise from a first-order
    # model of the throttle response. Throttle = feed-forward + kp_v * error;
    # when the car is faster than the target by more than ``brake_band`` the
    # throttle is set to ``brake_throttle`` (negative values brake hard).
    'use_odom_speed': True,
    'kp_v': 0.02,
    'brake_band': 0.2,
    'brake_throttle': -0.08,
    'speed_model_tau': 0.35,
    # Recovery: if the front is closer than ``emergency_dist``, or the car
    # is blocked (front below ``block_dist``) and has stopped, it backs up
    # for ``reverse_time`` seconds at ``reverse_throttle`` steering away from
    # the obstacle, then resumes.
    'emergency_dist': 0.25,
    'block_dist': 0.6,
    'reverse_throttle': -0.08,
    'reverse_time': 0.8,
    # If no scan arrives for this long the car is stopped (s).
    'watchdog_timeout': 0.5,
}


def clamp(value, low, high):
    return low if value < low else high if value > high else value


class WallFollowController:
    """Pure-numpy controller. Keeps no ROS state so it can be unit tested."""

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.derivative = 0.0
        self.prev_steer = 0.0
        self.v_est = 0.0
        self.v_meas = None
        self.reverse_left = 0.0
        self.reverse_steer = 0.0
        self.last = {}

    # ------------------------------------------------------------------ scan

    @staticmethod
    def clean_ranges(ranges, range_min, range_max):
        """Replace invalid readings (NaN, inf, below range_min) with range_max."""
        r = np.asarray(ranges, dtype=float)
        bad = ~np.isfinite(r) | (r < range_min) | (r <= 0.0)
        r = np.where(bad, range_max, r)
        return np.minimum(r, range_max)

    @staticmethod
    def index_for(angle_deg, angle_min, angle_inc, n):
        idx = int(round((math.radians(angle_deg) - angle_min) / angle_inc))
        return clamp(idx, 0, n - 1)

    def range_at(self, r, angle_deg, angle_min, angle_inc):
        """Robust range at an angle: mean of the window around the beam."""
        n = len(r)
        w = int(self.p['beam_window'])
        i = self.index_for(angle_deg, angle_min, angle_inc, n)
        lo, hi = max(0, i - w), min(n, i + w + 1)
        return float(np.mean(r[lo:hi]))

    def sector(self, r, lo_deg, hi_deg, angle_min, angle_inc):
        n = len(r)
        lo = self.index_for(lo_deg, angle_min, angle_inc, n)
        hi = self.index_for(hi_deg, angle_min, angle_inc, n)
        if hi < lo:
            lo, hi = hi, lo
        return lo, hi

    # -------------------------------------------------------------- geometry

    def projected_wall_distance(self, r, side, angle_min, angle_inc):
        """Distance to the wall on ``side`` (+1 left, -1 right) projected
        ``lookahead`` metres ahead of the car.

        Uses the two-beam method: beam ``b`` perpendicular to the car and beam
        ``a`` rotated ``theta`` towards the front. ``alpha`` is the angle
        between the car heading and the wall; it is negative when the car is
        heading towards that wall.
        """
        theta = math.radians(self.p['theta_deg'])
        cap = self.p['max_side_range']
        b = min(self.range_at(r, side * 90.0, angle_min, angle_inc), cap)
        a = min(self.range_at(r, side * (90.0 - self.p['theta_deg']), angle_min, angle_inc), cap)
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        dist = b * math.cos(alpha)
        future = dist + self.p['lookahead'] * math.sin(alpha)
        return future, dist, alpha

    def extend_disparities(self, sec, angle_inc):
        """Disparity extender: wherever neighbouring beams jump by more than
        ``disparity_threshold`` the nearer range is copied over the far side
        for the angular width of the car's safety radius, so a beam that
        merely grazes a wall edge is no longer seen as free."""
        p = self.p
        out = sec.copy()
        n = len(sec)
        jumps = np.flatnonzero(np.abs(np.diff(sec)) > p['disparity_threshold'])
        for i in jumps:
            if sec[i] < sec[i + 1]:
                d, start, step = sec[i], i + 1, 1
            else:
                d, start, step = sec[i + 1], i, -1
            width = int(math.asin(min(1.0, p['gap_bubble_radius'] / max(d, 1e-3))) / angle_inc) + 1
            for k in range(width):
                j = start + step * k
                if j < 0 or j >= n:
                    break
                if out[j] > d:
                    out[j] = d
        return out

    def gap_direction(self, r, angle_min, angle_inc):
        """Angle (rad, +left) of the deepest free direction in the forward
        sector after disparity extension and a safety bubble around the
        nearest obstacle."""
        p = self.p
        lo, hi = self.sector(r, -p['gap_sector_deg'], p['gap_sector_deg'], angle_min, angle_inc)
        sec = np.minimum(r[lo:hi + 1], p['gap_max_range'])
        sec = self.extend_disparities(sec, angle_inc)
        # Safety bubble around the closest point.
        i_min = int(np.argmin(sec))
        d_min = max(sec[i_min], 1e-3)
        half = int(math.atan2(p['gap_bubble_radius'], d_min) / angle_inc)
        sec[max(0, i_min - half):i_min + half + 1] = 0.0
        depth = sec.max()
        deep = np.flatnonzero(sec >= p['gap_depth_fraction'] * depth)
        # Largest contiguous run of deep beams, then its centre.
        splits = np.flatnonzero(np.diff(deep) > 1) + 1
        runs = np.split(deep, splits)
        best = max(runs, key=len)
        i_gap = int(best[len(best) // 2])
        angle = angle_min + (lo + i_gap) * angle_inc
        return angle, depth

    # --------------------------------------------------------------- control

    def compute(self, ranges, angle_min, angle_inc, range_min, range_max, dt):
        """Return (throttle, steering) commands in [-1, 1] for one scan."""
        p = self.p
        r = self.clean_ranges(ranges, range_min, range_max)
        dt = clamp(dt, 1e-3, 0.2)

        left_future, left_dist, left_alpha = self.projected_wall_distance(r, +1, angle_min, angle_inc)
        right_future, right_dist, right_alpha = self.projected_wall_distance(r, -1, angle_min, angle_inc)

        # Positive error always means "steer left".
        mode = str(p['wall']).lower()
        if mode == 'left':
            error = left_future - p['desired_distance']
        elif mode == 'right':
            error = p['desired_distance'] - right_future
        else:
            error = 0.5 * (left_future - right_future)

        # PID on the projected error with a filtered derivative.
        if self.prev_error is not None:
            raw = (error - self.prev_error) / dt
            a = dt / (p['derivative_tau'] + dt)
            self.derivative += a * (raw - self.derivative)
        self.prev_error = error
        self.integral = clamp(self.integral + error * dt, -p['integral_limit'], p['integral_limit'])
        wall_angle = p['kp'] * error + p['ki'] * self.integral + p['kd'] * self.derivative

        # Free distance ahead and the deepest free direction.
        lo, hi = self.sector(r, -p['front_cone_deg'], p['front_cone_deg'], angle_min, angle_inc)
        front = float(np.min(r[lo:hi + 1]))
        gap_angle, gap_depth = self.gap_direction(r, angle_min, angle_inc)

        # Blend wall following with gap following as the front closes up.
        w = p['gap_min_weight'] + (1.0 - p['gap_min_weight']) * clamp(1.0 - front / p['gap_full_dist'], 0.0, 1.0)
        steer_angle = (1.0 - w) * wall_angle + w * gap_angle
        steer = clamp(steer_angle / MAX_STEER_ANGLE, -1.0, 1.0)
        s = clamp(p['steer_smoothing'], 0.0, 0.99)
        steer = s * self.prev_steer + (1.0 - s) * steer

        # Speed target from stopping distance and cornering limit.
        v_stop = math.sqrt(max(0.0, 2.0 * p['brake_decel'] * (front - p['stop_margin'])))
        delta = abs(steer) * MAX_STEER_ANGLE
        radius = p['wheelbase'] / max(math.tan(delta), 1e-3)
        # Radius of the arc that reaches the gap point (pure pursuit).
        d_gap = min(gap_depth, p['gap_lookahead'])
        radius_gap = d_gap / max(2.0 * abs(math.sin(gap_angle)), 1e-3)
        v_corner = math.sqrt(p['lat_accel'] * min(radius, radius_gap))
        v_turn = math.sqrt(p['turn_speed'] ** 2 + 2.0 * p['brake_decel'] * max(front - p['turn_dist'], 0.0))
        v_target = clamp(min(v_stop, v_corner, v_turn), p['v_min'], p['v_max'])
        if front < p['stop_margin']:
            v_target = 0.0

        # Speed control: feed-forward plus proportional term on the measured
        # (or modelled) speed; brake when clearly above the target.
        v_now = self.v_meas if (p['use_odom_speed'] and self.v_meas is not None) else self.v_est
        throttle = v_target / p['speed_per_throttle'] + p['kp_v'] * (v_target - v_now)
        if v_now > v_target + p['brake_band']:
            throttle = p['brake_throttle']

        # Recovery: back out of a wall, steering away from the obstacle.
        if self.reverse_left > 0.0:
            self.reverse_left -= dt
            throttle = p['reverse_throttle']
            steer = self.reverse_steer
        elif front < p['emergency_dist'] or (front < max(p['block_dist'], p['stop_margin']) and v_now < 0.5):
            self.reverse_left = p['reverse_time']
            self.reverse_steer = -1.0 if gap_angle > 0 else 1.0
            throttle = p['reverse_throttle']
            steer = self.reverse_steer
            self.integral = 0.0

        throttle = clamp(throttle, -1.0, 1.0)
        self.prev_steer = steer
        # First-order model of the vehicle speed for the brake decision.
        v_ss = max(throttle, 0.0) * p['speed_per_throttle']
        if throttle < 0.0 and self.reverse_left <= 0.0:
            v_ss = 0.0
        self.v_est += (v_ss - self.v_est) * dt / p['speed_model_tau']
        if p['use_odom_speed'] and self.v_meas is not None:
            self.v_est = self.v_meas

        self.last = {
            'error': error, 'front': front, 'left': left_dist, 'right': right_dist,
            'gap_deg': math.degrees(gap_angle), 'w': w, 'v_target': v_target, 'v_est': v_now,
            'steer': steer, 'throttle': throttle,
        }
        return throttle, steer


################################################################################


class WallFollowNode(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        # Declare every tunable as a parameter so it can be set from the CLI.
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.controller = WallFollowController(params)
        self.add_on_set_parameters_callback(self._on_parameters)

        qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_throttle = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', qos)
        self.pub_steering = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', qos)
        self.sub_lidar = self.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar', self._on_scan, qos)
        # Speedometer only: used for closed-loop speed control, never for steering.
        self.sub_odom = self.create_subscription(Odometry, '/autodrive/roboracer_1/odom', self._on_odom, qos)

        self._lock = threading.Lock()
        self._last_scan_time = None
        self._scan_count = 0
        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            'Wall follower ready (mode=%s). Waiting for LiDAR on /autodrive/roboracer_1/lidar' % params['wall'])

    # ------------------------------------------------------------ callbacks

    def _on_parameters(self, params):
        with self._lock:
            for param in params:
                if param.name in DEFAULTS:
                    self.controller.p[param.name] = param.value
                    self.get_logger().info('Set %s = %s' % (param.name, param.value))
        return SetParametersResult(successful=True)

    def _on_odom(self, msg):
        v = msg.twist.twist.linear
        with self._lock:
            self.controller.v_meas = math.sqrt(v.x * v.x + v.y * v.y)

    def _on_scan(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            dt = 0.025 if self._last_scan_time is None else now - self._last_scan_time
            self._last_scan_time = now
            self._scan_count += 1
            if len(msg.ranges) == 0:
                return
            throttle, steer = self.controller.compute(
                msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, dt)
        self._publish(throttle, steer)

    def _watchdog(self):
        with self._lock:
            last = self._last_scan_time
        if last is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - last > self.controller.p['watchdog_timeout']:
            self._publish(0.0, 0.0)

    def _report(self):
        with self._lock:
            d = dict(self.controller.last)
            count = self._scan_count
        if not d:
            return
        self.get_logger().info(
            'scans=%d err=%+.2f front=%.2f L=%.2f R=%.2f gap=%+.0f w=%.2f v=%.1f/%.1f steer=%+.2f thr=%+.3f' % (
                count, d['error'], d['front'], d['left'], d['right'], d['gap_deg'], d['w'],
                d['v_est'], d['v_target'], d['steer'], d['throttle']))

    def _publish(self, throttle, steer):
        t = Float32()
        s = Float32()
        t.data = float(throttle)
        s.data = float(steer)
        self.pub_throttle.publish(t)
        self.pub_steering.publish(s)


################################################################################


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Bring the car to a stop before leaving.
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
