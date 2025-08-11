#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryCalculator(Node):
    """
    Node ini menghitung dan mempublikasikan odometri robot berdasarkan data tick encoder.
    - Subscribe: /encoder1, /encoder2, /encoder3, /encoder4 (std_msgs/msg/Int32)
    - Publish: /odom (nav_msgs/msg/Odometry)
    - Publish: /rpm1, /rpm2, /rpm3, /rpm4 (std_msgs/msg/Float64)
    - Broadcast: Transformasi dari 'odom' ke 'base_link'
    """
    def __init__(self):
        super().__init__('odometry_calculator')

        # --- Parameter Robot ---
        self.WHEEL_RADIUS = 0.175 # meter
        self.WHEEL_BASE = 0.88    # meter (jarak antar roda kiri dan kanan, l2)
        self.ENCODER_PPR = 600.0  # Pulses Per Revolution

        # --- Variabel State ---
        self.current_ticks = [0, 0, 0, 0] # Ticks untuk [FL, FR, RL, RR]
        self.last_ticks = [0, 0, 0, 0]
        self.last_time = self.get_clock().now()

        # Posisi dan orientasi robot [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- Subscribers untuk Encoder ---
        # Asumsi pemetaan:
        # encoder1 -> Front Left (W_FL)
        # encoder2 -> Front Right (W_FR)
        # encoder3 -> Rear Left (W_RL)
        # encoder4 -> Rear Right (W_RR)
        self.create_subscription(Int32, '/encoder1', lambda msg: self.encoder_callback(msg, 0), 10)
        self.create_subscription(Int32, '/encoder2', lambda msg: self.encoder_callback(msg, 1), 10)
        self.create_subscription(Int32, '/encoder3', lambda msg: self.encoder_callback(msg, 2), 10)
        self.create_subscription(Int32, '/encoder4', lambda msg: self.encoder_callback(msg, 3), 10)

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.rpm_pubs = [
            self.create_publisher(Float64, '/rpm1', 10), # FL
            self.create_publisher(Float64, '/rpm2', 10), # FR
            self.create_publisher(Float64, '/rpm3', 10), # RL
            self.create_publisher(Float64, '/rpm4', 10)  # RR
        ]

        # --- TF Broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timer untuk loop perhitungan utama ---
        self.timer = self.create_timer(0.02, self.calculate_and_publish_odometry) # 50 Hz
        self.get_logger().info('Odometry Calculator Node has been started.')

    def encoder_callback(self, msg, wheel_index):
        """Menyimpan data tick encoder terbaru."""
        self.current_ticks[wheel_index] = msg.data

    def calculate_and_publish_odometry(self):
        """Fungsi utama yang dipanggil oleh timer untuk menghitung dan mempublikasikan data."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Hindari pembagian dengan nol jika dt sangat kecil atau pada iterasi pertama
        if dt <= 0.0:
            return

        # 1. Hitung delta ticks
        delta_ticks = [self.current_ticks[i] - self.last_ticks[i] for i in range(4)]

        # 2. Hitung kecepatan sudut roda (rad/s) dan RPM
        # rev_per_sec = (delta_ticks / PPR) / dt
        # rad_per_sec = rev_per_sec * 2 * pi
        # rpm = rev_per_sec * 60
        rad_per_sec_per_wheel = [(delta_ticks / self.ENCODER_PPR)/0.02]
        rpm_per_wheel = [(rad_per_sec_per_wheel * 60)]

        # Publikasikan RPM
        for i in range(4):
            rpm_msg = Float64()
            rpm_msg.data = rpm_per_wheel[i]
            self.rpm_pubs[i].publish(rpm_msg)

        # 3. Hitung kecepatan rata-rata sisi kiri dan kanan (berdasarkan kinematika)
        omega_fl, omega_fr, omega_rl, omega_rr = rad_per_sec_per_wheel
        
        # Kecepatan sudut rata-rata untuk sisi kiri dan kanan
        omega_left = (omega_fl + omega_rl) / 2.0
        omega_right = (omega_fr + omega_rr) / 2.0

        # Kecepatan linear untuk sisi kiri dan kanan
        v_left = omega_left * self.WHEEL_RADIUS
        v_right = omega_right * self.WHEEL_RADIUS

        # 4. Hitung kecepatan linear (v_x) dan angular (v_th) robot
        v_x = (v_right + v_left) / 2.0
        v_th = (v_right - v_left) / self.WHEEL_BASE

        # 5. Integrasi untuk menghitung posisi (Odometry)
        delta_x = v_x * math.cos(self.theta) * dt
        delta_y = v_x * math.sin(self.theta) * dt
        delta_th = v_th * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        
        # Normalisasi sudut theta agar tetap dalam rentang [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 6. Buat dan publikasikan pesan Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set posisi
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientasi (konversi dari euler ke quaternion)
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set kecepatan
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = v_th

        self.odom_pub.publish(odom_msg)

        # 7. Broadcast transformasi TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # 8. Update state untuk iterasi berikutnya
        self.last_ticks = self.current_ticks[:]
        self.last_time = current_time

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Konversi sudut Euler ke Quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr
        qw = cy * cp * cr + sy * sp * sr
        
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    odometry_calculator_node = OdometryCalculator()
    try:
        rclpy.spin(odometry_calculator_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_calculator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import math
# import time

# class OdometryPublisher(Node):
#     def __init__(self):
#         super().__init__('encoder_to_odom')

#         # Encoder subscriber
#         self.encoder1_sub = self.create_subscription(Int32, '/encoder1', self.encoder1_callback, 10)
#         self.encoder2_sub = self.create_subscription(Int32, '/encoder2', self.encoder2_callback, 10)
#         self.encoder3_sub = self.create_subscription(Int32, '/encoder3', self.encoder3_callback, 10)
#         self.encoder4_sub = self.create_subscription(Int32, '/encoder4', self.encoder4_callback, 10)

#         # Odometry publisher
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # RPM publishers per roda
#         self.rpm1_pub = self.create_publisher(Float32, '/rpm1', 10)
#         self.rpm2_pub = self.create_publisher(Float32, '/rpm2', 10)
#         self.rpm3_pub = self.create_publisher(Float32, '/rpm3', 10)
#         self.rpm4_pub = self.create_publisher(Float32, '/rpm4', 10)

#         # Encoder variables
#         self.encoder1 = 0
#         self.encoder2 = 0
#         self.encoder3 = 0
#         self.encoder4 = 0

#         self.last_encoder1 = 0
#         self.last_encoder2 = 0
#         self.last_encoder3 = 0
#         self.last_encoder4 = 0

#         # Robot position (x, y, theta)
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0

#         # Timer untuk update odometry (100ms)
#         self.last_time = self.get_clock().now()
#         self.timer = self.create_timer(0.1, self.update_odometry)

#         # Konstanta (ubah sesuai kebutuhan)
#         self.TICKS_PER_REV = 600.0            # encoder 600 PPR
#         self.WHEEL_RADIUS = 0.175             # meter (35 cm diameter)
#         self.ROBOT_WIDTH = 0.88               # meter (jarak antar roda kanan-kiri)
#         self.GEAR_RATIO = 11.0 / 20.0         # rasio gigi encoder terhadap roda

#         # Inisialisasi buffer RPM untuk smoothing (optional)
#         self.rpm_buffer = [0.0, 0.0, 0.0, 0.0]
#         self.smoothing_alpha = 0.5

#     def encoder1_callback(self, msg):
#         self.encoder1 = msg.data

#     def encoder2_callback(self, msg):
#         self.encoder2 = msg.data

#     def encoder3_callback(self, msg):
#         self.encoder3 = msg.data

#     def encoder4_callback(self, msg):
#         self.encoder4 = msg.data

#     def update_odometry(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds * 1e-9
#         if dt < 0.01:
#             return

#         # Ambil delta encoder tiap roda
#         delta1 = self.encoder1 - self.last_encoder1
#         delta2 = self.encoder2 - self.last_encoder2
#         delta3 = self.encoder3 - self.last_encoder3
#         delta4 = -(self.encoder4 - self.last_encoder4)  # koreksi arah encoder 4

#         # Simpan nilai sekarang untuk iterasi berikutnya
#         self.last_encoder1 = self.encoder1
#         self.last_encoder2 = self.encoder2
#         self.last_encoder3 = self.encoder3
#         self.last_encoder4 = self.encoder4
#         self.last_time = current_time

#         # Hitung jarak per roda
#         d1 = 2 * math.pi * self.WHEEL_RADIUS * ((delta1 / self.TICKS_PER_REV) * self.GEAR_RATIO)
#         d2 = 2 * math.pi * self.WHEEL_RADIUS * ((delta2 / self.TICKS_PER_REV) * self.GEAR_RATIO)
#         d3 = 2 * math.pi * self.WHEEL_RADIUS * ((delta3 / self.TICKS_PER_REV) * self.GEAR_RATIO)
#         d4 = 2 * math.pi * self.WHEEL_RADIUS * ((delta4 / self.TICKS_PER_REV) * self.GEAR_RATIO)

#         # Hitung odometry berdasarkan rata-rata roda kiri dan kanan (d1+d3 vs d2+d4)
#         d_left = (d1 + d3) / 2.0
#         d_right = (d2 + d4) / 2.0

#         d_center = (d_left + d_right) / 2.0
#         d_theta = (d_right - d_left) / self.ROBOT_WIDTH

#         # Update posisi global
#         self.x += d_center * math.cos(self.theta + d_theta / 2.0)
#         self.y += d_center * math.sin(self.theta + d_theta / 2.0)
#         self.theta += d_theta

#         # Buat dan publish pesan Odometry
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'

#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Konversi theta ke quaternion
#         qz = math.sin(self.theta / 2.0)
#         qw = math.cos(self.theta / 2.0)
#         odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

#         odom_msg.twist.twist.linear.x = d_center / dt
#         odom_msg.twist.twist.angular.z = d_theta / dt

#         self.odom_pub.publish(odom_msg)

#         # Hitung dan publish RPM per roda dengan smoothing
#         deltas = [delta1, delta2, delta3, delta4]
#         pubs = [self.rpm1_pub, self.rpm2_pub, self.rpm3_pub, self.rpm4_pub]

#         for i in range(4):
#             revolutions = (deltas[i] / self.TICKS_PER_REV) * self.GEAR_RATIO
#             rpm = (revolutions / dt) * 60.0
#             self.rpm_buffer[i] = self.smoothing_alpha * rpm + (1 - self.smoothing_alpha) * self.rpm_buffer[i]
#             msg = Float32()
#             msg.data = self.rpm_buffer[i]
#             pubs[i].publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdometryPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

