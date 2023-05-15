import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.move_turtle)
        self.twist_msg_ = Twist()

    def move_turtle(self):
                # move a tartaruga para a posição inicial da estrela
        self.twist_msg_.linear.x = 2.0
        self.twist_msg_.angular.z = 0.0
        self.publisher_.publish(self.twist_msg_)
        time.sleep(2.0)

        # desenha a primeira ponta da estrela
        self.twist_msg_.linear.x = 1.0
        self.twist_msg_.angular.z = math.radians(144)
        self.publisher_.publish(self.twist_msg_)
        time.sleep(2.0)

        # desenha as próximas pontas da estrela
        for i in range(4):
            self.twist_msg_.linear.x = 1.0
            self.twist_msg_.angular.z = math.radians(72)
            self.publisher_.publish(self.twist_msg_)
            time.sleep(2.0)

            self.twist_msg_.linear.x = 1.0
            self.twist_msg_.angular.z = math.radians(144)
            self.publisher_.publish(self.twist_msg_)
            time.sleep(2.0)

        # para a tartaruga
        self.twist_msg_.linear.x = 0.0
        self.twist_msg_.angular.z = 0.0
        self.publisher_.publish(self.twist_msg_)

def main(args=None):
    rclpy.init()
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
