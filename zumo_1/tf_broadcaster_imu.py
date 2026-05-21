#!/usr/bin/env python3
"""
tf_broadcaster_imu.py — ROS 2 (Humble)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Equivalente ROS 2 del tf_broadcaster_imu.py de FerJeffQ (ROS 1).

Suscribe /zumo_imu (sensor_msgs/Imu) y publica:
  - TF: odom → imu_link  (con el tiempo real del sistema, no del ESP32)
  - /odom (nav_msgs/Odometry) con la orientación del IMU

El timestamp del mensaje del ESP32 llega en 0 porque micro-ROS
aún no sincronizó el reloj. Este nodo lo reemplaza con
self.get_clock().now() — exactamente como FerJeffQ usa
rospy.Time.now() en su versión ROS 1.

Ubicación en el paquete:
  zumo_1/zumo_1/tf_broadcaster_imu.py

Registrar en setup.py → entry_points:
  'tf_broadcaster_imu = zumo_1.tf_broadcaster_imu:main',

Uso:
  ros2 run zumo_1 tf_broadcaster_imu
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import tf2_ros
import math


class TfBroadcasterImu(Node):

    def __init__(self):
        super().__init__('tf_broadcaster_imu')

        # ── TF broadcaster ───────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publisher /odom ──────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # ── Subscriber /zumo_imu ─────────────────────────────
        self.create_subscription(Imu, 'zumo_imu', self.imu_callback, 10)

        self.get_logger().info('tf_broadcaster_imu iniciado — /zumo_imu → TF odom→imu_link')

    def imu_callback(self, msg: Imu):
        # ── Tiempo real del sistema (no del ESP32) ────────────
        # El ESP32 puede tener stamp en 0 si aún no sincronizó.
        # Usamos el reloj del PC — igual que rospy.Time.now()
        # en la versión ROS 1 de FerJeffQ.
        now = self.get_clock().now().to_msg()

        q = msg.orientation  # quaternión directo del DMP

        # ── Publicar TF odom → imu_link ──────────────────────
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'imu_link'

        # Posición: fija en origen (sin odometría de ruedas aún)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Orientación: quaternión del DMP (ya normalizado)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(t)

        # ── Publicar /odom ────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'imu_link'

        odom.pose.pose.orientation = q

        # Velocidad angular del IMU → twist del odom
        odom.twist.twist.angular.x = msg.angular_velocity.x
        odom.twist.twist.angular.y = msg.angular_velocity.y
        odom.twist.twist.angular.z = msg.angular_velocity.z

        # Covarianza de orientación del DMP (diagonal)
        odom.pose.covariance[21] = 0.01  # roll
        odom.pose.covariance[28] = 0.01  # pitch
        odom.pose.covariance[35] = 0.01  # yaw

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcasterImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
