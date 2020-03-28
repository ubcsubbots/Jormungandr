
# Created By: Edward Chang
# Created On: March, 14th 2020
# Key logging system


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import tf



def main():

    publisher = rospy.Publisher("/decision/output", Pose)
    rospy.init_node('keyboard_control', anonymous=True)
    while not rospy.is_shutdown():
        msg = Pose()
        x = True
        while x = True:
            print("Enter [x,y,z,phi,theta,psi] with no spaces, such as [5,2,1,0,0,3.14]")
            inputStr = input()
            split = inputStr.split(",")
            if (len(split) != 6):
                continue
            try:
                msg.position.x = int(split[0])
            except ValueError:
                print("X Integer conversion error")
                continue
            try:
                msg.position.y = int(split[1])
            except ValueError:
                print("Y Integer conversion error")
                continue
            try:
                msg.position.z = int(split[2])
            except ValueError:
                print("Z Integer conversion error")
                continue
            try:
                roll = int(split[3])
            except ValueError:
                print("Phi Integer conversion error")
                continue
            try:
                pitch = int(split[4])
            except ValueError:
                print("Pitch Integer conversion error")
                continue
            try:
                yaw = int(split[5])
            except ValueError:
                print("Yaw Integer conversion error")
                continue
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]

            x = False
        publisher.publish(msg)
        print("Message published")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass