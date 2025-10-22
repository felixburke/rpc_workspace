import rclpy
from rclpy.node import Node
test
from geometry_msgs.msg import Twist
import time

import math
import numpy as np


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")  # Name deines Nodes
        self.get_logger().info("Node gestartet!")

        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        time.sleep(1)

        self.turnAround(90)
        self.move_forward(2.0, 1.0)
        self.turnAround(-45)
        self.move_forward(2.0, 1.0 / math.sqrt(2))
        self.turnAround(-90)
        self.move_forward(2.0, 1.0 / math.sqrt(2))
        self.turnAround(-45)
        self.move_forward(2.0, 1.0)
        self.turnAround(-45 * 3)
        self.move_forward(2.0, 1.0 * math.sqrt(2))
        self.turnAround(-45 * 3)
        self.move_forward(2.0, 1.0)
        self.turnAround(-45 * 3)
        self.move_forward(2.0, 1.0 * math.sqrt(2))
        self.turnAround(45 * 3)
        self.move_forward(2.0, 1.0)

    def move_forward(self, speed, duration):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.get_logger().info("Vorwärts...")
        start = time.time()
        while time.time() - start < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)

    def turnAround(self, degrees):
        self.get_logger().info(f"Drehung: {degrees}")

        angle_rad = math.radians(degrees)
        sign = np.sign(angle_rad)
        angular_speed = math.pi / 2  # rad/s

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = sign * angular_speed

        seconds = abs(angle_rad) / angular_speed

        start = time.time()
        while time.time() - start < seconds:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        # stop rotation
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)  # ROS initialisieren
    node = MyNode()  # Node erzeugen
    rclpy.spin(node)  # Node "laufen lassen"
    node.destroy_node()  # Aufräumen
    rclpy.shutdown()  # ROS beenden


if __name__ == "__main__":
    main()
