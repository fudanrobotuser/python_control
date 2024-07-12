#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def imu_callback(data):
    orientation_q = data.orientation
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w)
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    rospy.loginfo("Roll: {:.2f} Pitch: {:.2f} Yaw: {:.2f}".format(roll, pitch, yaw))

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass
