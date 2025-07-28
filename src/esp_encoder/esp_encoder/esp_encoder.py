#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """
    Mengubah sudut Euler (dalam radian) menjadi Quaternion.
    Yaw (theta) adalah rotasi pada sumbu Z.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class EspEncoder(Node):
    def __init__(self):
        super().__init__('esp_encoder_node')

        # ==================== PARAMETER FISIK ROBOT (WAJIB DIUBAH) ====================
        self.declare_parameter('wheel_radius', 0.175)      # Jari-jari roda dalam meter
        self.declare_parameter('wheel_base', 0.88)         # Jarak antar roda kiri & kanan (meter)
        self.declare_parameter('ticks_per_rev', 2400) # Jumlah pulsa encoder per putaran
        
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().integer_value
        # ==============================================================================
        
        # Inisialisasi variabel state odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Variabel untuk menyimpan data terakhir
        self.last_ticks = {'fl': 0, 'fr': 0, 'bl': 0, 'br': 0}
        self.first_message_received = False
        self.last_time = self.get_clock().now()

        # Buat subscriber, publisher, dan TF broadcaster
        self.encoder_subscription = self.create_subscription(
            Int32MultiArray,
            'encoder_counts',
            self.encoder_callback,
            10)
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Odometry Publisher Node (4 Roda) telah dimulai.')
        self.get_logger().info(f'Parameter: Radius={self.wheel_radius}m, Base={self.wheel_base}m, Ticks/Rev={self.ticks_per_rev}')

    def encoder_callback(self, msg):
        """
        Callback yang dijalankan setiap ada data encoder baru,
        untuk menghitung perubahan pose (posisi & orientasi).
        """
        current_time = self.get_clock().now()
        
        # Mapping data encoder ke roda. Sesuaikan jika perlu.
        # Asumsi: [depan-kiri, depan-kanan, belakang-kiri, belakang-kanan]
        current_ticks = {
            'fl': msg.data[0], 'fr': msg.data[1],
            'bl': msg.data[2], 'br': msg.data[3]
        }

        if not self.first_message_received:
            self.last_ticks = current_ticks
            self.last_time = current_time
            self.first_message_received = True
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        # Hitung perubahan pulsa dan rata-ratakan untuk sisi kiri dan kanan
        delta_ticks = {key: current_ticks[key] - self.last_ticks[key] for key in current_ticks}
        delta_left_ticks = (delta_ticks['fl'] + delta_ticks['bl']) / 2.0
        delta_right_ticks = (delta_ticks['fr'] + delta_ticks['br']) / 2.0

        # Konversi pulsa ke jarak (meter)
        dist_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        dist_left = delta_left_ticks * dist_per_tick
        dist_right = delta_right_ticks * dist_per_tick

        # Hitung perubahan total jarak dan orientasi
        delta_dist = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        # Update pose (posisi x, y, dan orientasi theta)
        # Menggunakan metode Runge-Kutta orde 2 untuk akurasi lebih baik
        self.x += delta_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Update nilai terakhir untuk iterasi berikutnya
        self.last_ticks = current_ticks
        self.last_time = current_time
        
        # Panggil fungsi untuk mempublikasikan hasil
        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        """Membangun dan mempublikasikan pesan Odometry dan transformasi TF."""
        
        # --- 1. Publikasi Pesan Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Atur posisi
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Atur orientasi (dikonversi ke quaternion)
        odom_msg.pose.pose.orientation = euler_to_quaternion(self.theta)
        
        self.odom_publisher.publish(odom_msg)

        # --- 2. Publikasi Transformasi TF (odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Atur translasi dan rotasi
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    esp_encoder = EspEncoder()
    rclpy.spin(esp_encoder)
    esp_encoder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
