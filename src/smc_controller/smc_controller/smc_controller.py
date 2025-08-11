#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import numpy as np
from nav_msgs.msg import Path


def euler_from_quaternion(quaternion):
    # ... (fungsi konversi dari respons sebelumnya) ...
    x = quaternion.x; y = quaternion.y; z = quaternion.z; w = quaternion.w
    t3 = +2.0 * (w * z + x * y); t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class SmcControllerNode(Node):
    def __init__(self):
        super().__init__('smc_trajectory_tracker')
        
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        # --- Parameter Kontrol (BISA ANDA TUNE) ---
        # self.V_CONSTANT = 1.0  # Kecepatan linear konstan (m/s)
        # self.Ky = 2.0          # Gain untuk error 'y'
        # self.K_omega = 0.5     # Gain switching untuk 'omega'
        # self.PHI = 0.5         # Boundary layer untuk fungsi sat()

        self.V_CONSTANT = 1.2  # Kecepatan linear konstan (m/s)
        self.Ky = 6          # Gain untuk error 'y'
        self.K_omega = 1.0     # Gain switching untuk 'omega'
        self.PHI = 2.0         # Boundary layer untuk fungsi sat()

        # --- State & Target ---
        self.initial_x = 10.0
        self.initial_y = 0.0

        # Inisialisasi posisi saat ini dengan posisi awal
        self.current_x = self.initial_x
        self.current_y = self.initial_y
        self.current_theta = 2.0

        self.trajectory_received = False
        self.final_target_x = None

        # --- Subscribers & Publisher ---
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.path_sub = self.create_subscription(Path, '/line_trajectory', self.path_callback, 10)  # Path subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # --- Loop Kontrol ---
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        self.get_logger().info('SMC Controller Node has been started.')

    # def odom_callback(self, msg):
    #     self.current_x = msg.pose.pose.position.x # <<< UBAH INI
    #     self.current_y = msg.pose.pose.position.y # <<< UBAH INI

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        self.current_x = self.initial_x + odom_x
        self.current_y = self.initial_y + odom_y

    def imu_callback(self, msg):
        self.current_theta = euler_from_quaternion(msg.orientation)

    def path_callback(self, msg):
        # <<< MODIFIKASI 2: Buka gerbang saat path diterima dan simpan titik akhir >>>
        if msg.poses and not self.trajectory_received:
            # Ambil koordinat x dari pose terakhir dalam path
            self.final_target_x = msg.poses[-1].pose.position.x
            self.trajectory_received = True # Buka gerbang!
            self.get_logger().info(f"Trajectory received with {len(msg.poses)} points. Final target x: {self.final_target_x}. Starting control.")

    # def path_callback(self, msg):
    #     # Ambil titik pertama dari path
    #     if msg.poses:
    #         self.target_x = msg.poses[0].pose.position.x
    #         self.target_y = msg.poses[0].pose.position.y
    #         self.get_logger().info(f"Path received: Target x={self.target_x}, y={self.target_y}")


    def sat_function(self, s, phi):
        if s > phi: return 1.0
        elif s < -phi: return -1.0
        else: return s / phi

    def control_loop(self):
        if not self.trajectory_received:
            return
    
        # Target untuk garis lurus y=0
        y_d = 0.0
        
        # 1. Hitung error y
        y_error = self.current_y - y_d
        
        # 2. Hitung theta_d (orientasi target dinamis)
        theta_d = -math.atan(self.Ky * y_error)
        
        # 3. Hitung sliding surface 's'
        s = self.current_theta - theta_d

        # 4. Hitung omega (ω) menggunakan hukum kontrol SMC
        # Komponen pertama (equivalent control)
        u_eq = -(self.Ky * self.V_CONSTANT * math.sin(self.current_theta)) / (1 + (self.Ky * y_error)**2)
        # Komponen kedua (switching control)
        u_sw = -self.K_omega * self.sat_function(s, self.PHI)
        
        omega = u_eq + u_sw
        
        # 5. Atur kecepatan linear dan angular
        v = self.V_CONSTANT
        # Hentikan robot jika sudah mencapai ujung lintasan (misal x > 9.8)
        if self.current_x > 49.8:
            v = 0.0
            omega = 0.0
            
        # Berhenti jika robot sudah mencapai atau sedikit melewati titik x terakhir
        if self.current_x >= self.final_target_x - 0.2: # Beri buffer kecil 20 cm
            v = 0.0
            omega = 0.0

        # 6. Publikasikan perintah
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = omega
        self.cmd_pub.publish(twist_msg)
        
        # Tambahkan self.current_x dan self.current_y ke dalam log
        self.get_logger().info(
            f'Pose: (x={self.current_x:.2f}, y={self.current_y:.2f}) | '
            f'y_err: {y_error:.2f}, s: {s:.2f}, ω: {omega:.2f}, v: {v:.2f}'
        )

    def stop_robot(self):
        self.get_logger().info('Sending stop command to robot.')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)

def main(args=None):
    # --- INI ADALAH POLA MAIN YANG SUDAH BENAR DAN STANDAR ---
    rclpy.init(args=args)
    
    smc_controller_node = SmcControllerNode()
    
    try:
        rclpy.spin(smc_controller_node)
    except KeyboardInterrupt:
        # Ini akan dieksekusi saat Anda menekan Ctrl+C
        smc_controller_node.get_logger().info('Keyboard interrupt, stopping robot...')
    finally:
        # Pastikan robot berhenti dan node dihancurkan dengan benar
        smc_controller_node.stop_robot()
        smc_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


