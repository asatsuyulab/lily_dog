#!/usr/bin/env python
# license removed for brevity
import rospy
from can_msgs.msg import Frame

def PC_CAN_Node():
    pub = rospy.Publisher('CAN_robot_command', Frame, queue_size=10)
    rospy.init_node('PC_CAN_Node', anonymous=True)
    r = rospy.Rate(10) # 10hz
    msg = Frame()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.id = 0x000
        msg.dlc = 8
        msg.data = "00000000"
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        PC_CAN_Node()
    except rospy.ROSInterruptException: pass