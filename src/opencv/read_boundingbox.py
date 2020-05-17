from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState
from darknet_ros_msgs import BoudingBox
from darknet
import sys

class ReadStates:

    def __init__(self):

        self.yaw_sub = rospy.Subscriber("/darknet/boundingbox", JointState, self.read_yaw)


    def read_yaw(self, msg):
        pitch = msg.position[0]
        pitch_d = msg.velocity[0]
        pitch_e = msg.effort[0]

        roll = msg.position[1]
        roll_d = msg.velocity[1]
        roll_e = msg.effort[1]

        yaw = msg.position[2]
        yaw_d = msg.velocity[2]
        yaw_e = msg.effort[2]
        print("\t Pos \t Vel \t Effort")
        print ("Pitch:" , "\t", pitch, "\t", pitch_d, "\t", pitch_e)
        print ("Roll:" , "\t", roll, "\t", roll_d, "\t", roll_e)
        print ("Yaw:" , "\t", yaw, "\t", yaw_d, "\t", yaw_e)


def main(args):
    ic =ReadStates()
    rospy.init_node('read_states', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)