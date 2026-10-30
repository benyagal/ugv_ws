#!/usr/bin/env python3
"""Gripper ROS action server for the mission planner.

Action: /gripper (ugv_interface/action/Behavior), goal.command:
  pickup  -> gripper_repl DOWNINUPOUT (felvétel, a gripper fent marad)
  empty   -> gripper_repl PUSHPULL    (tároló ürítés)
Cancelling the goal = gripper_repl STOP. Service /gripper/stop (Trigger)
does the same without needing the goal handle.

The hardware sequences are imported from gripper_repl, so timings and pins
stay in one place. Difference from gripper_repl: the Rosmaster serial port is
opened WRITE-ONLY (no create_receive_threading()), same as ugv_driver, so this
node can run alongside ugv_bringup (the only allowed reader of the port).
Each set_pwm_servo() is a single small write(), which the tty layer does not
interleave with ugv_driver's writes.

Do NOT run gripper_repl at the same time as this node (both drive the same
servos/relays, and gripper_repl starts its own serial reader thread).

Usage: ros2 run ugv_tools gripper_server
"""
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

import Jetson.GPIO as GPIO
from Rosmaster_Lib import Rosmaster
from ugv_interface.action import Behavior

from ugv_tools import gripper_repl as hw

COMMANDS = {
    'pickup': lambda bot: hw.s2_down_in_up_out(bot),
    'empty': lambda bot: hw.relay_push_pull(),
}


class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value

        hw.setup_gpio()
        self.bot = Rosmaster(car_type=hw.CAR_TYPE, com=port)
        hw.s1_stop(self.bot)
        hw.relay_stop()
        # Same assumption as gripper_repl: S2/S3 rest at DOWN at power-on.
        hw.current_s23_angle = hw.S2_DOWN

        self._lock = threading.Lock()
        self._busy = False

        cb = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, Behavior, 'gripper',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb)
        self.create_service(Trigger, 'gripper/stop', self._on_stop, callback_group=cb)
        self.get_logger().info(
            f"Gripper server ready on {port} (write-only), commands: {', '.join(COMMANDS)}")

    def _on_goal(self, request):
        cmd = request.command.strip().lower()
        if cmd not in COMMANDS:
            self.get_logger().warn(f"Unknown gripper command: '{request.command}'")
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn(f"Gripper busy, '{cmd}' rejected")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        hw.stop_event.set()
        return CancelResponse.ACCEPT

    def _on_stop(self, request, response):
        hw.stop_event.set()
        hw.all_stop(self.bot)
        response.success = True
        response.message = 'gripper stopped'
        return response

    def _execute(self, goal_handle):
        cmd = goal_handle.request.command.strip().lower()
        result = Behavior.Result()
        hw.stop_event.clear()
        self.get_logger().info(f"'{cmd}' start")
        error = None
        try:
            COMMANDS[cmd](self.bot)
        except Exception as e:  # GPIO / serial failure mid-sequence
            error = e
        finally:
            with self._lock:
                self._busy = False

        if error is not None or hw.stop_event.is_set():
            hw.all_stop(self.bot)
            result.result = False
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f"'{cmd}' cancelled")
            else:
                goal_handle.abort()
                self.get_logger().error(f"'{cmd}' aborted: {error or 'stopped'}")
            return result

        goal_handle.succeed()
        result.result = True
        self.get_logger().info(f"'{cmd}' done")
        return result

    def shutdown(self):
        hw.stop_event.set()
        try:
            hw.all_stop(self.bot)
        finally:
            GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
