#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray


def talker():
    pub1 = rospy.Publisher('pid_set', Int16MultiArray, queue_size=10)
    pub2 = rospy.Publisher('set_vel', Float32MultiArray, queue_size=10)

    rospy.init_node('pid_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pid_values = Int16MultiArray()
    set_values = Float32MultiArray()
    p = 2.0
    i = 1.0
    d = 0.0
    sp_left = 0.0
    sp_right = 0.0
    pid_values.data.append(p)
    pid_values.data.append(i)
    pid_values.data.append(d)
    set_values.data.append(sp_left)
    set_values.data.append(sp_right)
    while not rospy.is_shutdown():

        pub1.publish(pid_values)
        pub2.publish(set_values)
        rospy.loginfo(pid_values)
        rospy.loginfo(set_values)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass