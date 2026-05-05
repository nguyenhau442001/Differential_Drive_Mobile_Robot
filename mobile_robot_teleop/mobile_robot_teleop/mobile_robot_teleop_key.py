#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import sys
import select
import os

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

MAX_LIN_VEL = 10.0
MAX_ANG_VEL = 25.0
LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Differential-Drive Mobile Robot!!!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(lin, ang):
    return 'currently:\tlinear vel %s\t angular vel %s ' % (lin, ang)


def make_simple_profile(output, input_val, slop):
    if input_val > output:
        output = min(input_val, output + slop)
    elif input_val < output:
        output = max(input_val, output - slop)
    return output


def constrain(val, low, high):
    return max(low, min(high, val))


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'w':
                target_linear_vel = constrain(target_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = constrain(target_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = constrain(target_angular_vel + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = constrain(target_angular_vel - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            elif key == '\x03':
                break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()
            control_linear_vel = make_simple_profile(
                control_linear_vel, target_linear_vel, LIN_VEL_STEP_SIZE / 2.0)
            twist.linear.x = control_linear_vel

            control_angular_vel = make_simple_profile(
                control_angular_vel, target_angular_vel, ANG_VEL_STEP_SIZE / 2.0)
            twist.angular.z = control_angular_vel

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
