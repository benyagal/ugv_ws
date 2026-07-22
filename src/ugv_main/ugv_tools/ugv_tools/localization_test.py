"""
Lokalizációs teszt node — 1.5 x 2 méteres tesztkörnyezethez.

Három tesztet végez:
  1. STATIKUS TESZT   — robot áll, mennyit driftel a becsült pozíció
  2. LINEÁRIS TESZT   — robot egyenesen megy A→B, visszamegy B→A
  3. NÉGYZET TESZT    — robot bejár egy 1x1 méteres négyzetet

Minden teszt végén kiírja:
  - Pozícióhiba (m)
  - Max eltérés (m)
  - Átlagos hiba (m)
  - Szórás (m)

Indítás (miután a bringup_localization_uwb.launch.py fut):
  ros2 run ugv_tools localization_test --ros-args -p test:=static
  ros2 run ugv_tools localization_test --ros-args -p test:=linear
  ros2 run ugv_tools localization_test --ros-args -p test:=square
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import math
import time
import statistics


class LocalizationTest(Node):
    def __init__(self):
        super().__init__('localization_test')

        self.declare_parameter('test', 'static')
        self.declare_parameter('duration', 30.0)       # statikus teszt hossza (s)
        self.declare_parameter('linear_speed', 0.15)   # m/s
        self.declare_parameter('angular_speed', 0.5)   # rad/s

        self.test_type = self.get_parameter('test').value
        self.duration = self.get_parameter('duration').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Pozíció adatok
        self.current_x = None
        self.current_y = None
        self.uwb_x = None
        self.uwb_y = None
        self.positions = []        # (timestamp, x, y) lista
        self.uwb_positions = []    # UWB mért pozíciók

        # Subscriberek
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/global', self._on_odom, 10)
        self.uwb_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/uwb/pose', self._on_uwb, 10)

        # Publisher (mozgásparancs)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(f'Teszt típusa: {self.test_type}')
        self.get_logger().info('Várakozás az első odom adatra...')

        # Indítás 2 másodperc várakozás után
        self.start_timer = self.create_timer(2.0, self._start_test)

    def _on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.positions.append((
            time.time(),
            self.current_x,
            self.current_y
        ))

    def _on_uwb(self, msg: PoseWithCovarianceStamped):
        self.uwb_x = msg.pose.pose.position.x
        self.uwb_y = msg.pose.pose.position.y
        self.uwb_positions.append((
            time.time(),
            self.uwb_x,
            self.uwb_y
        ))

    def _start_test(self):
        self.start_timer.cancel()

        if self.current_x is None:
            self.get_logger().warn('Még nincs odom adat! Elindult a bringup?')
            return

        self.get_logger().info(f'Teszt indul. Kezdő pozíció: ({self.current_x:.3f}, {self.current_y:.3f})')
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.positions.clear()
        self.uwb_positions.clear()

        if self.test_type == 'static':
            self._run_static_test()
        elif self.test_type == 'linear':
            self._run_linear_test()
        elif self.test_type == 'square':
            self._run_square_test()
        else:
            self.get_logger().error(f'Ismeretlen teszt: {self.test_type}')

    # ─── 1. STATIKUS TESZT ───────────────────────────────────────────────────
    def _run_static_test(self):
        self.get_logger().info(f'STATIKUS TESZT: {self.duration}s-ig áll a robot.')
        self.get_logger().info('Ne mozgasd a robotot!')

        self.static_timer = self.create_timer(self.duration, self._finish_static)

    def _finish_static(self):
        self.static_timer.cancel()
        self._stop_robot()

        if len(self.positions) < 5:
            self.get_logger().error('Nincs elég adat!')
            return

        xs = [p[1] for p in self.positions]
        ys = [p[2] for p in self.positions]

        drift_x = max(xs) - min(xs)
        drift_y = max(ys) - min(ys)
        total_drift = math.sqrt(drift_x**2 + drift_y**2)

        end_x = self.positions[-1][1]
        end_y = self.positions[-1][2]
        end_error = math.sqrt((end_x - self.start_x)**2 + (end_y - self.start_y)**2)

        self.get_logger().info('─' * 50)
        self.get_logger().info('STATIKUS TESZT EREDMÉNY:')
        self.get_logger().info(f'  Mért adatpontok száma : {len(self.positions)}')
        self.get_logger().info(f'  Drift X irányban      : {drift_x*100:.1f} cm')
        self.get_logger().info(f'  Drift Y irányban      : {drift_y*100:.1f} cm')
        self.get_logger().info(f'  Teljes drift          : {total_drift*100:.1f} cm')
        self.get_logger().info(f'  Végső pozícióhiba     : {end_error*100:.1f} cm')
        self.get_logger().info('─' * 50)
        self.get_logger().info('ÉRTÉKELÉS:')
        if total_drift < 0.05:
            self.get_logger().info('  ✓ KIVÁLÓ  (<5 cm drift)')
        elif total_drift < 0.15:
            self.get_logger().info('  ~ ELFOGADHATÓ (5-15 cm drift)')
        else:
            self.get_logger().warn('  ✗ GYENGE  (>15 cm drift) — EKF kovariancia hangolás szükséges')
        self.get_logger().info('─' * 50)

        self._log_uwb_stats()
        rclpy.shutdown()

    # ─── 2. LINEÁRIS TESZT ───────────────────────────────────────────────────
    def _run_linear_test(self):
        self.get_logger().info('LINEÁRIS TESZT: előre 1m, vissza 1m.')
        self.get_logger().info('Ügyelj hogy 1m szabad hely legyen előre!')

        self._move_straight(1.0, callback=self._linear_halfway)

    def _linear_halfway(self):
        halfway_x = self.current_x
        halfway_y = self.current_y
        dist = math.sqrt((halfway_x - self.start_x)**2 + (halfway_y - self.start_y)**2)
        self.get_logger().info(f'Félúton: becsült megtett táv = {dist*100:.1f} cm (várt: 100 cm)')
        time.sleep(0.5)
        self._move_straight(-1.0, callback=self._finish_linear)

    def _finish_linear(self):
        self._stop_robot()
        end_x = self.current_x
        end_y = self.current_y
        return_error = math.sqrt((end_x - self.start_x)**2 + (end_y - self.start_y)**2)

        self.get_logger().info('─' * 50)
        self.get_logger().info('LINEÁRIS TESZT EREDMÉNY:')
        self.get_logger().info(f'  Visszatérési hiba: {return_error*100:.1f} cm')
        self.get_logger().info(f'  Kezdő: ({self.start_x:.3f}, {self.start_y:.3f})')
        self.get_logger().info(f'  Végső: ({end_x:.3f}, {end_y:.3f})')
        self.get_logger().info('ÉRTÉKELÉS:')
        if return_error < 0.05:
            self.get_logger().info('  ✓ KIVÁLÓ  (<5 cm visszatérési hiba)')
        elif return_error < 0.15:
            self.get_logger().info('  ~ ELFOGADHATÓ (5-15 cm)')
        else:
            self.get_logger().warn('  ✗ GYENGE  (>15 cm) — odometria kalibrálás szükséges')
        self.get_logger().info('─' * 50)

        self._log_uwb_stats()
        rclpy.shutdown()

    # ─── 3. NÉGYZET TESZT ────────────────────────────────────────────────────
    def _run_square_test(self):
        self.get_logger().info('NÉGYZET TESZT: 1x1 méteres négyzet bejárása.')
        self.get_logger().info('Ügyelj hogy 1x1m szabad terület legyen!')

        self._square_step = 0
        self._do_square_step()

    def _do_square_step(self):
        steps = [
            ('előre', 1.0, None),
            ('fordul', None, math.pi / 2),
            ('előre', 1.0, None),
            ('fordul', None, math.pi / 2),
            ('előre', 1.0, None),
            ('fordul', None, math.pi / 2),
            ('előre', 1.0, None),
            ('fordul', None, math.pi / 2),
        ]

        if self._square_step >= len(steps):
            self._finish_square()
            return

        label, dist, angle = steps[self._square_step]
        self._square_step += 1
        self.get_logger().info(f'Négyzet lépés {self._square_step}: {label}')

        if dist is not None:
            self._move_straight(dist, callback=self._do_square_step)
        else:
            self._rotate(angle, callback=self._do_square_step)

    def _finish_square(self):
        self._stop_robot()
        end_x = self.current_x
        end_y = self.current_y
        close_error = math.sqrt((end_x - self.start_x)**2 + (end_y - self.start_y)**2)

        self.get_logger().info('─' * 50)
        self.get_logger().info('NÉGYZET TESZT EREDMÉNY:')
        self.get_logger().info(f'  Záróhiba (vissza a startra): {close_error*100:.1f} cm')
        self.get_logger().info(f'  Kezdő: ({self.start_x:.3f}, {self.start_y:.3f})')
        self.get_logger().info(f'  Végső: ({end_x:.3f}, {end_y:.3f})')
        self.get_logger().info('ÉRTÉKELÉS:')
        if close_error < 0.10:
            self.get_logger().info('  ✓ KIVÁLÓ  (<10 cm záróhiba)')
        elif close_error < 0.25:
            self.get_logger().info('  ~ ELFOGADHATÓ (10-25 cm)')
        else:
            self.get_logger().warn('  ✗ GYENGE  (>25 cm) — TRACK_WIDTH kalibrálás szükséges')
        self.get_logger().info('─' * 50)

        self._log_uwb_stats()
        rclpy.shutdown()

    # ─── SEGÉDFÜGGVÉNYEK ─────────────────────────────────────────────────────
    def _move_straight(self, distance_m, callback):
        speed = self.linear_speed if distance_m > 0 else -self.linear_speed
        duration = abs(distance_m) / self.linear_speed
        twist = Twist()
        twist.linear.x = speed
        self.cmd_pub.publish(twist)
        self.create_timer(duration, lambda: self._stop_and_call(callback))

    def _rotate(self, angle_rad, callback):
        speed = self.angular_speed if angle_rad > 0 else -self.angular_speed
        duration = abs(angle_rad) / self.angular_speed
        twist = Twist()
        twist.angular.z = speed
        self.cmd_pub.publish(twist)
        self.create_timer(duration, lambda: self._stop_and_call(callback))

    def _stop_and_call(self, callback):
        self._stop_robot()
        time.sleep(0.3)
        callback()

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _log_uwb_stats(self):
        if len(self.uwb_positions) < 5:
            self.get_logger().warn('Kevés UWB adat érkezett a teszt során.')
            return

        # UWB frissítési frekvencia
        times = [p[0] for p in self.uwb_positions]
        intervals = [times[i+1] - times[i] for i in range(len(times)-1)]
        avg_freq = 1.0 / statistics.mean(intervals) if intervals else 0

        self.get_logger().info('UWB STATISZTIKA:')
        self.get_logger().info(f'  UWB adatpontok száma  : {len(self.uwb_positions)}')
        self.get_logger().info(f'  Átlagos frekvencia    : {avg_freq:.1f} Hz')
        if avg_freq < 5.0:
            self.get_logger().warn('  ⚠ Alacsony UWB frekvencia (<5 Hz) — anchor konfiguráció ellenőrzés!')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
