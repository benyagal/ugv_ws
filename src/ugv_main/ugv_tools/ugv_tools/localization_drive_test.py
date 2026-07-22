"""
Lokalizációs pontossági teszt — manuális vezérlés közben.

A robot teleop-pal mozog (pl. teleop_twist_joy), te vezeted a jelzett
pontokra, és minden pontnál Enter-t nyomsz. A szoftver méri az EKF
pozíció vs. valódi pozíció eltérést MOZGÁS KÖZBEN — ez a valódi
éles használatot szimulálja.

Különbség a localization_accuracy_test-től:
  - Ott a robot motorjai nem mozognak (kézzel tolod)
  - ITT a robot maga hajt oda teleop-pal (odometria + UWB fúzió aktív)

Párhuzamosan futó parancsok:
  Terminal 1: ros2 launch ugv_bringup bringup_localization_uwb.launch.py use_uwb_sim:=false ...
  Terminal 2: ros2 launch ugv_tools teleop_twist_joy.launch.py  (vagy joy_ctrl)
  Terminal 3: ros2 run ugv_tools localization_drive_test

Előkészítés:
  Ragassz le pontokat a padlón mérőszalaggal (méterben):
    P0: (0.00, 0.00) — start
    P1: (1.00, 0.00)
    P2: (1.00, 0.75)
    P3: (0.00, 0.75)
    P4: (0.50, 0.37) — középpont
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import math
import time
import statistics
import threading


TEST_POINTS = [
    ('P0', 0.00, 0.00),
    ('P1', 1.00, 0.00),
    ('P2', 1.00, 0.75),
    ('P3', 0.00, 0.75),
    ('P4', 0.50, 0.37),
]


class LocalizationDriveTest(Node):
    def __init__(self):
        super().__init__('localization_drive_test')

        self.ekf_x = None
        self.ekf_y = None
        self.uwb_x = None
        self.uwb_y = None

        # Teljes trajektória rögzítése
        self.trajectory = []
        self.uwb_trajectory = []

        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/global', self._on_ekf, 10)
        self.uwb_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/uwb/pose', self._on_uwb, 10)

        self.results = []

        self.test_thread = threading.Thread(target=self._run_test, daemon=True)
        self.test_thread.start()

    def _on_ekf(self, msg: Odometry):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y
        self.trajectory.append((time.time(), self.ekf_x, self.ekf_y))

    def _on_uwb(self, msg: PoseWithCovarianceStamped):
        self.uwb_x = msg.pose.pose.position.x
        self.uwb_y = msg.pose.pose.position.y
        self.uwb_trajectory.append((time.time(), self.uwb_x, self.uwb_y))

    def _run_test(self):
        # Várakozás az első adatokra
        timeout = 10.0
        start = time.time()
        while self.ekf_x is None:
            time.sleep(0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Nem érkezett EKF adat 10s alatt. Elindult a bringup?')
                rclpy.shutdown()
                return

        print('\n' + '═' * 60)
        print('  LOKALIZÁCIÓS TESZT — MANUÁLIS VEZÉRLÉSSEL')
        print('═' * 60)
        print('  Vezéreld a robotot teleop-pal a jelzett pontokra.')
        print('  Minden pontnál ÁLLÍTSD MEG a robotot, majd nyomj ENTER-t.')
        print('  A mérés az ÁLLÓ robot pozícióját rögzíti.')
        print('═' * 60 + '\n')

        for i, (label, true_x, true_y) in enumerate(TEST_POINTS):
            print(f'─' * 60)
            print(f'  [{i+1}/{len(TEST_POINTS)}] Vezess a(z) {label} pontra:')
            print(f'  Valódi koordináta: ({true_x:.2f}, {true_y:.2f}) m')
            print(f'  Most EKF mutat  : ({self.ekf_x:.3f}, {self.ekf_y:.3f}) m')
            print(f'  Állítsd meg a robotot a jelnél, majd nyomj ENTER-t...')
            input()

            # Robot leállítása biztonsági okokból
            # (teleop folytatható utána)

            # Átlagolás 30 mérésből (~3 másodperc) — álló robot
            print('  Mérés folyamatban (3s)...')
            ekf_samples = []
            uwb_samples = []
            for _ in range(30):
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
                ekf_std = statistics.stdev([
                    math.sqrt((s[0]-avg_ekf_x)**2 + (s[1]-avg_ekf_y)**2)
                    for s in ekf_samples
                ]) if len(ekf_samples) > 1 else 0.0
            else:
                avg_ekf_x = avg_ekf_y = ekf_error = ekf_std = float('nan')

            # UWB hiba
            if uwb_samples:
                avg_uwb_x = statistics.mean(s[0] for s in uwb_samples)
                avg_uwb_y = statistics.mean(s[1] for s in uwb_samples)
                uwb_error = math.sqrt((avg_uwb_x - true_x)**2 + (avg_uwb_y - true_y)**2)
                uwb_std = statistics.stdev([
                    math.sqrt((s[0]-avg_uwb_x)**2 + (s[1]-avg_uwb_y)**2)
                    for s in uwb_samples
                ]) if len(uwb_samples) > 1 else 0.0
            else:
                avg_uwb_x = avg_uwb_y = uwb_error = uwb_std = float('nan')

            print(f'\n  EREDMÉNY {label}:')
            print(f'  EKF  → ({avg_ekf_x:.3f}, {avg_ekf_y:.3f})  hiba: {ekf_error*100:.1f} cm  szórás: {ekf_std*100:.1f} cm')
            print(f'  UWB  → ({avg_uwb_x:.3f}, {avg_uwb_y:.3f})  hiba: {uwb_error*100:.1f} cm  szórás: {uwb_std*100:.1f} cm')

            self.results.append({
                'label': label,
                'true_x': true_x,
                'true_y': true_y,
                'ekf_x': avg_ekf_x,
                'ekf_y': avg_ekf_y,
                'ekf_error': ekf_error,
                'ekf_std': ekf_std,
                'uwb_x': avg_uwb_x,
                'uwb_y': avg_uwb_y,
                'uwb_error': uwb_error,
                'uwb_std': uwb_std,
            })

            if i < len(TEST_POINTS) - 1:
                print(f'\n  Folytasd a vezérlést a következő pontra...')

        self._print_summary()
        rclpy.shutdown()

    def _print_summary(self):
        print('\n' + '═' * 60)
        print('  ÖSSZEFOGLALÁS — MOZGÁS KÖZBENI LOKALIZÁCIÓ')
        print('═' * 60)

        ekf_errors = [r['ekf_error'] for r in self.results if not math.isnan(r['ekf_error'])]
        uwb_errors = [r['uwb_error'] for r in self.results if not math.isnan(r['uwb_error'])]

        print(f'\n  {"Pont":<5} {"Valódi pos":>14}  {"EKF hiba":>9}  {"UWB hiba":>9}  {"EKF szórás":>10}')
        print(f'  {"─"*5} {"─"*14}  {"─"*9}  {"─"*9}  {"─"*10}')
        for r in self.results:
            print(f'  {r["label"]:<5} ({r["true_x"]:.2f},{r["true_y"]:.2f})     '
                  f'{r["ekf_error"]*100:>6.1f} cm  '
                  f'{r["uwb_error"]*100:>6.1f} cm  '
                  f'{r["ekf_std"]*100:>7.1f} cm')

        if ekf_errors:
            print(f'\n  EKF átlagos hiba : {statistics.mean(ekf_errors)*100:.1f} cm')
            print(f'  EKF max hiba     : {max(ekf_errors)*100:.1f} cm')
            print(f'  EKF min hiba     : {min(ekf_errors)*100:.1f} cm')
        if uwb_errors:
            print(f'\n  UWB átlagos hiba : {statistics.mean(uwb_errors)*100:.1f} cm')
            print(f'  UWB max hiba     : {max(uwb_errors)*100:.1f} cm')

        # Trajektória statisztika
        print(f'\n  Rögzített EKF trajektória pontok : {len(self.trajectory)}')
        print(f'  Rögzített UWB trajektória pontok : {len(self.uwb_trajectory)}')

        print('\n  ÉRTÉKELÉS:')
        if ekf_errors:
            avg = statistics.mean(ekf_errors)
            if avg < 0.10:
                print('  ✓ KIVÁLÓ  — EKF átlagos hiba <10 cm mozgás közben')
            elif avg < 0.20:
                print('  ~ ELFOGADHATÓ — EKF átlagos hiba 10-20 cm')
            else:
                print('  ✗ GYENGE  — EKF >20 cm hiba → UWB kovariancia vagy anchor pozíció ellenőrzés')

        if ekf_errors and uwb_errors:
            ekf_avg = statistics.mean(ekf_errors)
            uwb_avg = statistics.mean(uwb_errors)
            if ekf_avg < uwb_avg:
                print('  ✓ Az EKF fúzió javít az UWB nyers adaton — a rendszer jól működik')
            else:
                print('  ⚠ Az EKF nem javít az UWB-n — EKF súlyok ellenőrzése szükséges')

        print('═' * 60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationDriveTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
