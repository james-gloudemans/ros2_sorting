import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

from jackal_teleop.controller import Controller

JACKAL_MAX_LIN_VEL = 1.0  # from data sheet
JACKAL_MAX_ANG_VEL = 2.0  # blind guess

CONTROLLER_MAX_VALUE = 32767  # triggers and joysticks give 2B signed ints


class JackalTeleop(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.node = rclpy.create_node("jackal_teleop")
        self.qos = QoSProfile(depth=10)
        self.pub = self.node.create_publisher(Twist, "cmd_vel", self.qos)
        self.msg = Twist()

    ###############################################################
    # Not implemented
    ###############################################################

    def on_x_press(self):
        pass

    def on_x_release(self):
        pass

    def on_triangle_press(self):
        pass

    def on_triangle_release(self):
        pass

    def on_circle_press(self):
        pass

    def on_circle_release(self):
        pass

    def on_square_press(self):
        pass

    def on_square_release(self):
        pass

    def on_L1_press(self):
        pass

    def on_L1_release(self):
        pass

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        pass

    def on_up_arrow_press(self):
        pass

    def on_up_down_arrow_release(self):
        pass

    def on_down_arrow_press(self):
        pass

    def on_left_arrow_press(self):
        pass

    def on_left_right_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        pass

    def on_R3_up(self, value):
        pass

    def on_R3_down(self, value):
        pass

    def on_R3_left(self, value):
        pass

    def on_R3_right(self, value):
        pass

    def on_R3_y_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        pass

    def on_R3_x_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        pass

    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        pass

    def on_R3_release(self):
        """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        pass

    def on_L3_y_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        pass

    def on_L3_release(self):
        """L3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        pass

    def on_L3_up(self, value):
        pass

    def on_L3_down(self, value):
        pass

    def on_options_press(self):
        pass

    def on_options_release(self):
        pass

    def on_share_press(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    def on_share_release(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    def on_playstation_button_press(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    def on_playstation_button_release(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    ###############################################################
    # Implemented buttons, overriden from class Controller
    ###############################################################

    def on_L2_press(self, value):
        value += CONTROLLER_MAX_VALUE
        self.msg.linear.x = -(value / (2 * CONTROLLER_MAX_VALUE)) * JACKAL_MAX_LIN_VEL
        self.pub.publish(self.msg)

    def on_L2_release(self):
        self.msg.linear.x = 0.0
        self.pub.publish(self.msg)

    def on_R2_press(self, value):
        value += CONTROLLER_MAX_VALUE
        self.msg.linear.x = (value / (2 * CONTROLLER_MAX_VALUE)) * JACKAL_MAX_LIN_VEL
        self.pub.publish(self.msg)

    def on_R2_release(self):
        self.msg.linear.x = 0.0
        self.pub.publish(self.msg)

    def on_L3_left(self, value):
        self.msg.angular.z = -(value / CONTROLLER_MAX_VALUE) * JACKAL_MAX_ANG_VEL
        self.pub.publish(self.msg)

    def on_L3_right(self, value):
        self.msg.angular.z = -(value / CONTROLLER_MAX_VALUE) * JACKAL_MAX_ANG_VEL
        self.pub.publish(self.msg)

    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)

    def on_L3_press(self):
        """L3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        self.msg = Twist()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    controller = JackalTeleop(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()


if __name__ == "__main__":
    main()
