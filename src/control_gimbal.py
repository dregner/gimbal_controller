
import rospy
import sys

from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
from std_msgs.msg import Int16

class gimbal_control:
    def __init__(self):
        self.pos_x_sub = rospy.Subscriber("/pixel_x", Int16, self.read_px_x)
        self.pos_y_sub = rospy.Subscriber("/pixel_y", Int16, self.read_px_y)
        self.joint_state_read = rospy.Subscriber("/gimbal/joint_states", JointControllerState, self.read_js)
        self.pub_yaw = rospy.Publisher("gimbal/yaw_positon_controller/command")

        self.pitch, self.roll, self.yaw     # position values from Joint State
        self.pos_x, self.pos_y              # pixel position of the center of object tracked

    def read_js(self, msg):
        self.pitch = msg.position[0]
        pitch_d = msg.velocity[0]
        pitch_e = msg.effort[0]

        self.roll = msg.position[1]
        roll_d = msg.velocity[1]
        roll_e = msg.effort[1]

        self.yaw = msg.position[2]
        yaw_d = msg.velocity[2]
        yaw_e = msg.effort[2]
        print("\t Pos \t Vel \t Effort")
        print ("Pitch:" , "\t", self.pitch, "\t", pitch_d, "\t", pitch_e)
        print ("Roll:" , "\t", self.roll, "\t", roll_d, "\t", roll_e)
        print ("Yaw:" , "\t", self.yaw, "\t", yaw_d, "\t", yaw_e)

    def read_px_x(self,data):
        self.pos_x = data

    def read_px_y(self,data):
        self.pos_y = data

def main(args):
    gc = gimbal_control()
    rospy.init_node('control_gimbal', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Closing")

if name == '__main__':
    main(sys.argv)