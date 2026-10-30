#!/usr/bin/env python3
"""Docking onto the charger (plan section 12).

/dock action (ugv_interface/Behavior):
  'dock'    Start at the pre-dock pose (dock_distance in front of the charger,
            facing it). Measures the resting battery voltage, then drives
            straight forward: fast_speed, slow_speed for the last slow_distance.
            Charging = voltage at least charge_rise above the resting value.
            Seen while driving -> stop and confirm standing still. Reached
            dock_distance + max_overshoot or stuck -> stop and check anyway
            (the charger may switch on with a delay). Not charging -> back off
            retreat_distance and retry (attempts in total).
  'undock'  Reverse undock_distance straight off the charger.

The V-shaped guide on the charger removes the remaining lateral / heading
error, so the heading is only held before the slow zone.
Publishes straight to cmd_vel: call it only while Nav2 is idle.
"""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from ugv_interface.action import Behavior

PERIOD = 0.05  # s, control loop and cmd_vel rate (driver timeout is 0.5 s)
PROGRESS_STEP = 0.02  # m, less than this within stuck_time = stuck
TF_LOSS_LIMIT = 1.0  # s


class Canceled(Exception):
    pass


class DockServer(Node):
    def __init__(self):
        super().__init__('dock_server')
        p = self.declare_parameter
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('robot_base_frame', 'base_footprint').value
        self.dock_distance = p('dock_distance', 1.5).value
        self.max_overshoot = p('max_overshoot', 0.25).value
        self.fast_speed = p('fast_speed', 0.15).value
        self.slow_speed = p('slow_speed', 0.10).value
        self.slow_distance = p('slow_distance', 0.5).value
        self.heading_gain = p('heading_gain', 1.0).value
        self.max_heading_rate = p('max_heading_rate', 0.3).value
        self.baseline_time = p('baseline_time', 2.0).value
        self.charge_rise = p('charge_rise', 0.3).value
        self.charge_hold = p('charge_hold', 0.5).value
        self.confirm_time = p('confirm_time', 3.0).value
        self.stuck_time = p('stuck_time', 3.0).value
        self.retreat_distance = p('retreat_distance', 0.5).value
        self.attempts = p('attempts', 3).value
        self.undock_distance = p('undock_distance', 0.6).value
        self.undock_speed = p('undock_speed', 0.15).value
        self.timeout = p('timeout', 120.0).value

        self._volts = deque(maxlen=2000)
        self._vlock = threading.Lock()
        self._busy = False
        self._handle = None
        self._tf_lost = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Float32, 'voltage', self._on_voltage, 50)
        ActionServer(self, Behavior, '/dock', execute_callback=self._execute,
                     goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'Dock server ready (dock_distance {self.dock_distance} m, '
            f'{self.fast_speed}/{self.slow_speed} m/s, charge_rise {self.charge_rise} V)')

    # ------------------------------------------------------------------ inputs

    def _on_voltage(self, msg):
        if msg.data > 0.1:  # 0 = no reading from the board
            with self._vlock:
                self._volts.append((time.monotonic(), msg.data))

    def _median_voltage(self, window):
        now = time.monotonic()
        with self._vlock:
            vals = sorted(v for t, v in self._volts if now - t <= window)
        return vals[len(vals) // 2] if len(vals) >= 3 else None

    def _pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    # ------------------------------------------------------------------ action

    def _on_goal(self, request):
        if self._busy or request.command.strip().lower() not in ('dock', 'undock'):
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, handle):
        self._handle = handle
        command = handle.request.command.strip().lower()
        try:
            ok = self._dock() if command == 'dock' else self._undock()
        except Canceled:
            self._stop()
            self.get_logger().info(f'{command}: canceled')
            handle.canceled()
            return Behavior.Result(result=False)
        except Exception as e:
            self.get_logger().error(f'{command}: {type(e).__name__}: {e}')
            ok = False
        finally:
            self._stop()
            self._busy = False
        if ok:
            handle.succeed()
        else:
            handle.abort()
        return Behavior.Result(result=ok)

    # ----------------------------------------------------------------- docking

    def _dock(self):
        start = self._pose()
        if start is None:
            self.get_logger().error(f'dock: no TF {self.odom_frame} -> {self.base_frame}')
            return False
        self._hold_still(self.baseline_time)
        base = self._median_voltage(self.baseline_time)
        if base is None:
            self.get_logger().error('dock: no voltage readings')
            return False
        self.get_logger().info(f'dock: resting voltage {base:.2f} V, charging above '
                               f'{base + self.charge_rise:.2f} V')
        deadline = time.monotonic() + self.timeout
        for attempt in range(1, self.attempts + 1):
            reason = self._drive_in(start, base, deadline)
            if reason == 'timeout':
                break
            if self._confirm(base):
                self.get_logger().info(f'dock: charging (attempt {attempt}, {reason})')
                return True
            self.get_logger().warn(f'dock: attempt {attempt} ended ({reason}), not charging')
            if attempt < self.attempts and not self._back_off(start, deadline):
                break
        self.get_logger().error('dock: failed')
        return False

    def _drive_in(self, start, base, deadline):
        """Forward until charging / end of travel / stuck / timeout."""
        rise_since = None
        best, best_t = None, time.monotonic()
        while True:
            now = self._tick(deadline)
            if now is None:
                return 'timeout'
            along, yaw_err = self._progress(start)
            if along is None:
                continue
            if along >= self.dock_distance + self.max_overshoot:
                return 'end of travel'
            v = self._median_voltage(0.3)
            if v is not None and v >= base + self.charge_rise:
                rise_since = rise_since or now
                if now - rise_since >= self.charge_hold:
                    return 'voltage rise'
            else:
                rise_since = None
            if best is None or along > best + PROGRESS_STEP:
                best, best_t = along, now
            elif now - best_t > self.stuck_time:
                return 'stuck'
            slow = along >= self.dock_distance - self.slow_distance
            w = 0.0 if slow else max(-self.max_heading_rate,
                                     min(self.max_heading_rate, -self.heading_gain * yaw_err))
            self._publish(self.slow_speed if slow else self.fast_speed, w)

    def _confirm(self, base):
        self._hold_still(1.0 + self.confirm_time)  # 1 s for the motor sag to recover
        v = self._median_voltage(self.confirm_time)
        self.get_logger().info(f'dock: standing voltage {v if v is None else round(v, 2)} V')
        return v is not None and v >= base + self.charge_rise

    def _back_off(self, start, deadline):
        along, _ = self._progress(start)
        if along is None:
            return False
        return self._reverse(start, max(along - self.retreat_distance, 0.0), self.fast_speed, deadline)

    # --------------------------------------------------------------- undocking

    def _undock(self):
        start = self._pose()
        if start is None:
            self.get_logger().error(f'undock: no TF {self.odom_frame} -> {self.base_frame}')
            return False
        ok = self._reverse(start, -self.undock_distance, self.undock_speed,
                           time.monotonic() + self.timeout)
        log = self.get_logger().info if ok else self.get_logger().error
        log(f"undock: {'done' if ok else 'failed'}")
        return ok

    def _reverse(self, start, target_along, speed, deadline):
        """Straight backwards until the position along the start heading drops to target_along."""
        best, best_t = None, time.monotonic()
        while True:
            now = self._tick(deadline)
            if now is None:
                return False
            along, _ = self._progress(start)
            if along is None:
                continue
            if along <= target_along:
                return True
            if best is None or along < best - PROGRESS_STEP:
                best, best_t = along, now
            elif now - best_t > self.stuck_time:
                self.get_logger().warn('reverse: stuck')
                return False
            self._publish(-speed, 0.0)

    # ----------------------------------------------------------------- helpers

    def _tick(self, deadline):
        """One control period; raises Canceled, returns None on timeout."""
        time.sleep(PERIOD)
        if self._handle.is_cancel_requested or not rclpy.ok():
            raise Canceled()
        now = time.monotonic()
        if now > deadline:
            self.get_logger().error('timeout')
            return None
        return now

    def _progress(self, start):
        """(distance along the start heading, heading error) or (None, None) without TF."""
        pose = self._pose()
        if pose is None:
            self._stop()
            if self._tf_lost is None:
                self._tf_lost = time.monotonic()
            elif time.monotonic() - self._tf_lost > TF_LOSS_LIMIT:
                raise RuntimeError('TF lost')
            return None, None
        self._tf_lost = None
        x0, y0, yaw0 = start
        x, y, yaw = pose
        along = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
        yaw_err = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
        return along, yaw_err

    def _hold_still(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self._handle.is_cancel_requested:
                raise Canceled()
            self._stop()
            time.sleep(PERIOD)

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DockServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
