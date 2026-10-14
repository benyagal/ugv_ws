#!/usr/bin/env python3
"""Chickcheck2 mission planner (plan section 5, milestone M2).

Collects dead chickens from /dead_chicken/detections (PoseArray, map frame),
visits them nearest-first with Nav2, picks them up with the gripper server,
empties the storage at a fixed unload pose when full, and drives to the
pre-dock pose when the list is empty or the battery is low.

States: IDLE, PAUSED, GOTO_APPROACH, FINAL_APPROACH, PICKUP, GOTO_UNLOAD,
UNLOAD, GOTO_DOCK.

Placeholders (until implemented):
  - FINAL_APPROACH: no visual servo yet (/final_approach, plan 6.4), skipped.
  - Docking: only Nav2 to `predock_pose`; /dock, /undock and charging
    (plan 12.x) are not called. The mission ends there (IDLE).

Interfaces:
  sub  /dead_chicken/detections  geometry_msgs/PoseArray (map)
  sub  /global_costmap/costmap   nav_msgs/OccupancyGrid (approach pose check)
  sub  /voltage                  std_msgs/Float32
  act  navigate_to_pose          nav2_msgs/NavigateToPose
  act  /gripper                  ugv_interface/Behavior (pickup / empty)
  srv  /mission/start|pause|stop|reset  std_srvs/Trigger
  pub  /mission/status           std_msgs/String (JSON, 1 Hz)

pause: cancels navigation, a running gripper cycle is finished first.
stop:  cancels navigation and the gripper cycle.
reset: (only when IDLE) clears the chicken list and the storage counter.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from ugv_interface.action import Behavior

# Chicken states
NEW, ACTIVE, DONE, FAILED, SKIPPED = 'new', 'active', 'done', 'failed', 'skipped'

# Mission modes (set by the services)
IDLE, RUNNING, PAUSED = 'IDLE', 'RUNNING', 'PAUSED'

# Approach directions tried around a chicken, relative to the robot's side.
APPROACH_OFFSETS_DEG = [0, 45, -45, 90, -90, 135, -135, 180]

# Published costmap OccupancyGrid values: 99 = inscribed, 100 = lethal, -1 = unknown.
BLOCKED_COST = 99

# Voltage must stay below dock_voltage this long (filters sag under load).
LOW_VOLTAGE_HOLD = 10.0  # s


class Aborted(Exception):
    """Raised inside the worker when pause/stop interrupts an action."""


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.base_frame = p('robot_base_frame', 'base_footprint').value
        detections_topic = p('detections_topic', '/dead_chicken/detections').value
        costmap_topic = p('costmap_topic', '/global_costmap/costmap').value
        gripper_action = p('gripper_action', '/gripper').value
        self.dedupe_radius = p('dedupe_radius', 0.5).value
        self.approach_distance = p('approach_distance', 1.0).value
        self.approach_clearance = p('approach_clearance', 0.2).value
        self.nav_attempts = p('nav_attempts', 2).value
        self.chicken_timeout = p('chicken_timeout', 300.0).value
        self.gripper_timeout = p('gripper_timeout', 60.0).value
        self.storage_capacity = p('storage_capacity', 3).value
        self.dock_voltage = p('dock_voltage', 0.0).value
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.unload_pose = self._pose_param(p('unload_pose', [], any_type).value, 'unload_pose')
        self.predock_pose = self._pose_param(p('predock_pose', [], any_type).value, 'predock_pose')

        self._lock = threading.Lock()
        self._chickens = []
        self._next_id = 0
        self._current = None
        self._storage = 0
        self._mode = IDLE
        self._state = IDLE
        self._last_error = ''
        self._voltage = None
        self._low_since = None
        self._costmap = None
        self._shutdown = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cb = ReentrantCallbackGroup()
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self._gripper = ActionClient(self, Behavior, gripper_action, callback_group=cb)

        self.create_subscription(PoseArray, detections_topic, self._on_detections, 10)
        self.create_subscription(Float32, 'voltage', self._on_voltage, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, costmap_topic, self._on_costmap, latched)

        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.create_timer(1.0, self._publish_status)
        for name, handler in (('start', self._on_start), ('pause', self._on_pause),
                              ('stop', self._on_stop), ('reset', self._on_reset)):
            self.create_service(Trigger, f'/mission/{name}', handler, callback_group=cb)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"Mission planner ready (capacity {self.storage_capacity}, "
            f"unload_pose {self.unload_pose}, predock_pose {self.predock_pose}, "
            f"dock_voltage {self.dock_voltage or 'disabled'})")

    def _pose_param(self, value, name):
        if value is None or len(value) == 0:
            return None
        if len(value) != 3:
            raise ValueError(f'{name} must be [x, y, yaw_deg], got {list(value)}')
        return (float(value[0]), float(value[1]), math.radians(float(value[2])))

    # ------------------------------------------------------------------ inputs

    def _on_detections(self, msg):
        if msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Detections in frame '{msg.header.frame_id}' ignored (expected '{self.global_frame}')",
                throttle_duration_sec=10.0)
            return
        with self._lock:
            for pose in msg.poses:
                x, y = pose.position.x, pose.position.y
                if any(math.hypot(x - c['x'], y - c['y']) < self.dedupe_radius
                       for c in self._chickens):
                    continue
                self._next_id += 1
                self._chickens.append(
                    {'id': self._next_id, 'x': x, 'y': y, 'state': NEW, 'error': ''})
                self.get_logger().info(f'New dead chicken #{self._next_id} at ({x:.2f}, {y:.2f})')

    def _on_voltage(self, msg):
        self._voltage = msg.data
        if self.dock_voltage > 0.0 and msg.data < self.dock_voltage:
            if self._low_since is None:
                self._low_since = time.monotonic()
        else:
            self._low_since = None

    def _on_costmap(self, msg):
        self._costmap = msg

    # ---------------------------------------------------------------- services

    def _on_start(self, request, response):
        if self._mode == RUNNING:
            response.success, response.message = False, 'already running'
        else:
            self._last_error = ''
            self._mode = RUNNING
            response.success, response.message = True, 'started'
            self.get_logger().info('Mission started')
        return response

    def _on_pause(self, request, response):
        if self._mode != RUNNING:
            response.success, response.message = False, f'not running ({self._mode})'
        else:
            self._mode = PAUSED
            response.success, response.message = True, 'paused'
            self.get_logger().info('Mission paused')
        return response

    def _on_stop(self, request, response):
        self._mode = IDLE
        response.success, response.message = True, 'stopped'
        self.get_logger().info('Mission stopped')
        return response

    def _on_reset(self, request, response):
        if self._mode != IDLE or self._state != IDLE:
            response.success, response.message = False, 'stop the mission first'
            return response
        with self._lock:
            self._chickens.clear()
        self._storage = 0
        self._last_error = ''
        response.success, response.message = True, 'list and storage counter cleared'
        self.get_logger().info('Mission reset')
        return response

    def _publish_status(self):
        with self._lock:
            chickens = [dict(c) for c in self._chickens]
        status = {
            'mode': self._mode,
            'state': self._state,
            'current': self._current,
            'storage': self._storage,
            'storage_capacity': self.storage_capacity,
            'voltage': self._voltage,
            'last_error': self._last_error,
            'chickens': chickens,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------ worker

    def _run(self):
        while rclpy.ok() and not self._shutdown:
            if self._mode != RUNNING:
                self._state = self._mode
                time.sleep(0.2)
                continue
            try:
                self._step()
            except Aborted:
                self.get_logger().info(f'Interrupted ({self._mode})')
            except Exception as e:
                self._error(f'{type(e).__name__}: {e}')

    def _error(self, message):
        self.get_logger().error(f'{message} -> mission paused')
        self._last_error = message
        self._mode = PAUSED

    def _step(self):
        if self._storage >= self.storage_capacity:
            if not self._unload():
                self._error(self._last_error or 'unload failed')
            return
        if self._battery_low():
            self.get_logger().warn(
                f'Battery below {self.dock_voltage:.1f} V for {LOW_VOLTAGE_HOLD:.0f} s -> going home')
            self._go_home('low battery')
            return
        chicken = self._next_chicken()
        if chicken is None:
            self._go_home('chicken list empty')
            return
        self._process(chicken)

    def _battery_low(self):
        return self._low_since is not None and time.monotonic() - self._low_since >= LOW_VOLTAGE_HOLD

    def _next_chicken(self):
        robot = self._robot_xy()
        with self._lock:
            pending = [c for c in self._chickens if c['state'] == NEW]
        if not pending:
            return None
        if robot is None:
            return pending[0]
        return min(pending, key=lambda c: math.hypot(c['x'] - robot[0], c['y'] - robot[1]))

    def _process(self, c):
        c['state'] = ACTIVE
        self._current = c['id']
        self.get_logger().info(f"Chicken #{c['id']} at ({c['x']:.2f}, {c['y']:.2f})")
        deadline = time.monotonic() + self.chicken_timeout
        try:
            if not self._approach(c, deadline):
                return

            self._state = 'FINAL_APPROACH'
            self.get_logger().info('FINAL_APPROACH: placeholder (no visual servo yet), skipped')

            self._state = 'PICKUP'
            res = self._run_action(self._gripper, Behavior.Goal(command='pickup'),
                                   self.gripper_timeout, cancel_on_pause=False)
            if res != 'ok':
                self._finish(c, FAILED, f'pickup {res}')
                return
            self._storage += 1
            self._finish(c, DONE)
        except Exception:
            if c['state'] == ACTIVE:
                c['state'] = NEW
            raise
        finally:
            self._current = None

    def _finish(self, c, state, error=''):
        c['state'] = state
        c['error'] = error
        log = self.get_logger().info if state == DONE else self.get_logger().warn
        log(f"Chicken #{c['id']}: {state}{' (' + error + ')' if error else ''}"
            f" - storage {self._storage}/{self.storage_capacity}")

    def _approach(self, c, deadline):
        robot = self._robot_xy()
        if robot is None:
            raise RuntimeError(f'no TF {self.global_frame} -> {self.base_frame}')
        candidates = self._approach_candidates(c, robot)
        if not candidates:
            self._finish(c, SKIPPED, 'no free approach pose')
            return False
        res = 'failed'
        for i, pose in enumerate(candidates[:self.nav_attempts]):
            self._state = 'GOTO_APPROACH'
            res = self._navigate(pose, deadline - time.monotonic())
            if res == 'ok':
                return True
            self.get_logger().warn(f"Chicken #{c['id']}: approach {i + 1} {res}")
            if res == 'timeout':
                break
        self._finish(c, FAILED, f'navigation {res}')
        return False

    def _approach_candidates(self, c, robot):
        if self._costmap is None:
            self.get_logger().warn('No global costmap received, approach pose not checked')
        base = math.atan2(robot[1] - c['y'], robot[0] - c['x'])
        out = []
        for off in APPROACH_OFFSETS_DEG:
            a = base + math.radians(off)
            x = c['x'] + self.approach_distance * math.cos(a)
            y = c['y'] + self.approach_distance * math.sin(a)
            if self._is_free(x, y):
                yaw = math.atan2(-math.sin(a), -math.cos(a))  # facing the chicken
                out.append((x, y, yaw))
        return out

    def _is_free(self, x, y):
        grid = self._costmap
        if grid is None:
            return True
        info = grid.info
        res = info.resolution
        ci = int(math.floor((x - info.origin.position.x) / res))
        cj = int(math.floor((y - info.origin.position.y) / res))
        r = int(math.ceil(self.approach_clearance / res))
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                i, j = ci + di, cj + dj
                if not (0 <= i < info.width and 0 <= j < info.height):
                    return False
                v = grid.data[j * info.width + i]
                if v < 0 or v >= BLOCKED_COST:
                    return False
        return True

    def _unload(self):
        if self.unload_pose is None:
            self._last_error = 'storage full but unload_pose is not set'
            return False
        self._state = 'GOTO_UNLOAD'
        self.get_logger().info(f'Storage {self._storage}/{self.storage_capacity} -> unload')
        res = self._navigate(self.unload_pose, self.chicken_timeout)
        if res != 'ok':
            self._last_error = f'navigation to unload_pose {res}'
            return False
        self._state = 'UNLOAD'
        res = self._run_action(self._gripper, Behavior.Goal(command='empty'),
                               self.gripper_timeout, cancel_on_pause=False)
        if res != 'ok':
            self._last_error = f'empty {res}'
            return False
        self._storage = 0
        self.get_logger().info('Storage emptied')
        return True

    def _go_home(self, reason):
        if self._storage > 0:
            if self.unload_pose is None:
                self.get_logger().warn('unload_pose not set, storage not emptied')
            elif not self._unload():
                self._error(self._last_error)
                return
        if self.predock_pose is None:
            self.get_logger().warn('predock_pose not set, staying here')
        else:
            self._state = 'GOTO_DOCK'
            res = self._navigate(self.predock_pose, self.chicken_timeout)
            if res != 'ok':
                self._error(f'navigation to predock_pose {res}')
                return
            # TODO (plan 12.5): /dock action + wait for charging + /undock.
        self.get_logger().info(f'Mission finished ({reason})')
        self._mode = IDLE

    # ----------------------------------------------------------------- helpers

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _navigate(self, pose, timeout):
        x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f'Nav2 goal ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        return self._run_action(self._nav, goal, timeout, cancel_on_pause=True)

    def _should_abort(self, cancel_on_pause):
        if self._shutdown or self._mode == IDLE:
            return True
        return cancel_on_pause and self._mode == PAUSED

    @staticmethod
    def _wait(future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        return future.done()

    def _run_action(self, client, goal, timeout, cancel_on_pause):
        """Send a goal and block the worker until it ends.
        Returns 'ok' / 'failed' / 'canceled' / 'timeout' / 'rejected' / 'unavailable',
        raises Aborted on pause/stop."""
        if timeout <= 0.0:
            return 'timeout'
        if not client.wait_for_server(timeout_sec=2.0):
            return 'unavailable'
        send_future = client.send_goal_async(goal)
        if not self._wait(send_future, 5.0):
            return 'unavailable'
        handle = send_future.result()
        if not handle.accepted:
            return 'rejected'
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done():
            if self._should_abort(cancel_on_pause):
                handle.cancel_goal_async()
                self._wait(result_future, 5.0)
                raise Aborted()
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                self._wait(result_future, 5.0)
                return 'timeout'
            time.sleep(0.1)
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'ok'
        if status == GoalStatus.STATUS_CANCELED:
            return 'canceled'
        return 'failed'


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
