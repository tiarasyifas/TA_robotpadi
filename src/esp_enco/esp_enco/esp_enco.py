import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class EncoderToOdometry(Node):
    def __init__(self):
        super().__init__('encoder_to_odometry_node')

        # Subscriber untuk menerima data dari encoder
        self.subscription = self.create_subscription(
            Int32,
            'micro_ros_odom',  # Topik dari ESP32
            self.encoder_callback,
            10)
        
        # Publisher untuk data odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odometry', 10)

        # Variabel posisi encoder
        self.encoder_position = 0
        self.last_encoder_position = 0

        # Timer untuk memperbarui data odometry setiap 100ms
        self.timer = self.create_timer(0.1, self.timer_callback)

    def encoder_callback(self, msg):
        """Menerima data dari encoder dan memperbarui posisi"""
        self.encoder_position = msg.data

    def timer_callback(self):
        """Menghitung odometry dan mempublikasikan"""
        # Hitung perbedaan posisi encoder
        delta_position = self.encoder_position - self.last_encoder_position
        self.last_encoder_position = self.encoder_position

        # Konversi perbedaan posisi menjadi kecepatan linear
        linear_velocity = delta_position * 0.001  # misal, 0.001 meter per langkah encoder
        angular_velocity = 0.0  # jika menggunakan roda diferensial, bisa dihitung

        # Membuat pesan odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Posisi (dalam x, y)
        odom_msg.pose.pose.position.x = self.encoder_position * 0.001  # misal, 0.001 meter per langkah encoder
        odom_msg.pose.pose.position.y = 0.0  # sesuaikan dengan pengaturan koordinat Anda

        # Kecepatan linear dan angular
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish data odometry
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
