
# Created By: Edward Chang
# Created On: March, 14th 2020
# Key logging system


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose


def main():

    msg = Pose()
    publisher = rospy.Publisher("/decision/output", Pose)
    rospy.init_node('keyboard_control', anonymous=True)
    while not rospy.is_shutdown():

        print("X coordinate")
        x = None
        while x is None:
            x = input()
            if x == "quit":
                break
            try:
                msg.position.x = int(x)
            except ValueError:
                x = None
        if x == "quit":
            continue

        print("Y coordinate")
        y = None
        while y is None:
            y = input()
            if y == "quit":
                break
            try:
                msg.position.x = int(y)
            except ValueError:
                x = None
        if y == "quit":
            continue

        print("Z coordinate")
        z = None
        while z is None:
            z = input()
            if z == "quit":
                break
            try:
                msg.position.z = int(z)
            except ValueError:
                z = None
        if z == "quit":
            continue

        publisher.publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass