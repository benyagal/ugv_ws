"""
UWB lokalizációs pontossági teszt — kézi pozicionálással.

A robot motorjai NEM mozognak — te helyezed a robotot ismert pontokra.
A szoftver méri az EKF/UWB által mutatott pozíció vs. valódi pozíció eltérést.

Előkészítés:
  Ragassz le pontokat a padlón mérőszalaggal.
  Ajánlott elrendezés 1.5x2m-es területen:
    P0: (0.00, 0.00) — sarok/origó
    P1: (1.00, 0.00) — 1m előre
    P2: (1.00, 0.75) — jobbra
    P3: (0.00, 0.75) — bal sarok
    P4: (0.50, 0.37) — középpont

Indítás:
  ros2 run ugv_tools localization_accuracy_test

A teszt menet közbeni kimenet:
  Pont P1 — valódi: (1.00, 0.00) — EKF mutat: (0.97, 0.03) — hiba: 3.2 cm
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import sys
import statistics
import threading


# Ismert valódi pozíciók (méterben) — igény szerint módosítható
TEST_POINTS = [
    ('P0', 0.00, 0.00),
    ('P1', 1.00, 0.00),
    ('P2', 1.00, 0.75),
    ('P3', 0.00, 0.75),
    ('P4', 0.50, 0.37),
]


class LocalizationAccuracyTest(Node):
    def __init__(self):
        super().__init__('localization_accuracy_test')

        self.ekf_x = None
        self.ekf_y = None
        self.uwb_x = None
        self.uwb_y = None

        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/global', self._on_ekf, 10)
        self.uwb_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/uwb/pose', self._on_uwb, 10)

        self.results = []

        self.get_logger().info('UWB pontossági teszt betöltve.')
        self.get_logger().info('Várakozás adatokra...')

        # A tesztet külön szálon futtatjuk hogy az input() ne blokklja a ROS spinnt
        self.test_thread = threading.Thread(target=self._run_test, daemon=True)
        self.test_thread.start()

    def _on_ekf(self, msg: Odometry):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y

    def _on_uwb(self, msg: PoseWithCovarianceStamped):
        self.uwb_x = msg.pose.pose.position.x
        self.uwb_y = msg.pose.pose.position.y

    def _run_test(self):
        # Várakozás az első adatokra
        import time
        timeout = 10.0
        start = time.time()
        while self.ekf_x is None:
            time.sleep(0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Nem érkezett EKF adat 10s alatt. Elindult a bringup?')
                rclpy.shutdown()
                return

        print('\n' + '═' * 55)
        print('  UWB LOKALIZÁCIÓS PONTOSSÁGI TESZT')
        print('═' * 55)
        print('  Helyezd a robotot minden jelzett pontra,')
        print('  majd nyomj ENTER-t a mérés rögzítéséhez.')
        print('═' * 55 + '\n')

        for label, true_x, true_y in TEST_POINTS:
            print(f'─' * 55)
            print(f'  Pont: {label}  →  valódi pozíció: ({true_x:.2f}, {true_y:.2f}) m')
            print(f'  Helyezd a robotot erre a pontra, majd nyomj ENTER-t...')
            input()

            # Átlagolás 20 mérésből (~2 másodperc)
            ekf_samples = []
            uwb_samples = []
            for _ in range(20):
                if self.ekf_x is not None:
                    ekf_samples.append((self.ekf_x, self.ekf_y))
                if self.uwb_x is not None:
                    uwb_samples.append((self.uwb_x, self.uwb_y))
                time.sleep(0.1)

            # EKF hiba
            if ekf_samples:
                avg_ekf_x = statistics.mean(s[0] for s in ekf_samples)
                avg_ekf_y = statistics.mean(s[1] for s in ekf_samples)
                ekf_error = math.sqrt((avg_ekf_x - true_x)**2 + (avg_ekf_y - true_y)**2)
            else:
                avg_ekf_x = avg_ekf_y = ekf_error = float('nan')

            # UWB hiba
            if uwb_samples:
                avg_uwb_x = statistics.mean(s[0] for s in uwb_samples)
                avg_uwb_y = statistics.mean(s[1] for s in uwb_samples)
                uwb_error = math.sqrt((avg_uwb_x - true_x)**2 + (avg_uwb_y - true_y)**2)
            else:
                avg_uwb_x = avg_uwb_y = uwb_error = float('nan')

            print(f'  EKF  mutat: ({avg_ekf_x:.3f}, {avg_ekf_y:.3f}) m  →  hiba: {ekf_error*100:.1f} cm')
            print(f'  UWB  mutat: ({avg_uwb_x:.3f}, {avg_uwb_y:.3f}) m  →  hiba: {uwb_error*100:.1f} cm')

            self.results.append({
                'label': label,
                'true_x': true_x,
                'true_y': true_y,
                'ekf_x': avg_ekf_x,
                'ekf_y': avg_ekf_y,
                'ekf_error': ekf_error,
                'uwb_x': avg_uwb_x,
                'uwb_y': avg_uwb_y,
                'uwb_error': uwb_error,
            })

        self._print_summary()
        rclpy.shutdown()

    def _print_summary(self):
        print('\n' + '═' * 55)
        print('  ÖSSZEFOGLALÁS')
        print('═' * 55)

        ekf_errors = [r['ekf_error'] for r in self.results if not math.isnan(r['ekf_error'])]
        uwb_errors = [r['uwb_error'] for r in self.results if not math.isnan(r['uwb_error'])]

        print(f'\n  {"Pont":<6} {"Valódi":>14} {"EKF hiba":>10} {"UWB hiba":>10}')
        print(f'  {"─"*6} {"─"*14} {"─"*10} {"─"*10}')
        for r in self.results:
            print(f'  {r["label"]:<6} ({r["true_x"]:.2f},{r["true_y"]:.2f})   '
                  f'{r["ekf_error"]*100:>7.1f} cm  {r["uwb_error"]*100:>7.1f} cm')

        if ekf_errors:
            print(f'\n  EKF átlagos hiba : {statistics.mean(ekf_errors)*100:.1f} cm')
            print(f'  EKF max hiba     : {max(ekf_errors)*100:.1f} cm')
        if uwb_errors:
            print(f'\n  UWB átlagos hiba : {statistics.mean(uwb_errors)*100:.1f} cm')
            print(f'  UWB max hiba     : {max(uwb_errors)*100:.1f} cm')

        print('\n  ÉRTÉKELÉS:')
        if ekf_errors:
            avg = statistics.mean(ekf_errors)
            if avg < 0.10:
                print('  ✓ EKF KIVÁLÓ  (<10 cm átlagos hiba)')
            elif avg < 0.25:
                print('  ~ EKF ELFOGADHATÓ (10-25 cm)')
            else:
                print('  ✗ EKF GYENGE  (>25 cm) — EKF / UWB kovariancia hangolás szükséges')

        print('═' * 55 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationAccuracyTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
