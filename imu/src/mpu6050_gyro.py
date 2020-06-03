#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist
import time
from mpu6050 import *


def pub_imu(last_time, x_rotation, y_rotation):
	imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
	rate = rospy.Rate(10)  # 10hz

	while not rospy.is_shutdown():

		gyro_data = mpu.get_gyro()
		accel_data = mpu.get_accel()

		new_time = rospy.Time.now().to_sec()
		dt = new_time - last_time
		last_time = rospy.Time.now().to_sec()

		x_rotation = gyro_data['x'] * dt + x_rotation

		if x_rotation > 360:
			x_rotation -= 360
		if x_rotation < 0:
			x_rotation = 360 + x_rotation

		y_rotation = gyro_data['y'] * dt + y_rotation

		imu_msg.orientation.x = x_rotation
		imu_msg.orientation.y = y_rotation
		imu_msg.orientation.z = 0

		imu_msg.linear_acceleration.x = accel_data['x']
		imu_msg.linear_acceleration.y = accel_data['y']
		imu_msg.linear_acceleration.z = accel_data['z']

		imu_msg.angular_velocity.x = gyro_data['x']
		imu_msg.angular_velocity.y = gyro_data['y']
		imu_msg.angular_velocity.z = gyro_data['z']
		imu_pub.publish(imu_msg)

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('mpu6050_gyro', anonymous=True)
		mpu = MPU6050()
		mpu.initialize()

		# Set calibration data
        mpu.gyro_offs = {'x': -361, 'y': -233, 'z': -253}
		mpu.accel_offs = {'y': 3, 'x': 5546, 'z': 97}

		x_rotation = 0
		y_rotation = 0
		imu_msg = Imu()
		last_time = rospy.Time.now().to_sec()
		pub_imu(last_time, x_rotation, y_rotation)
	except rospy.ROSInterruptException:
		pass
