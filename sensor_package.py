#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: September 17, 2024
# Last modifiec: September 20, 2024
#
# Extracts data from all sensors in the Miro Sensor Package

# Misc useful python libraries
import math
import os
import numpy as np

# Robot specific libraries
import rospy
from sensor_msgs.msg import JointState
import miro2 as miro

class see_touch:

    # TICK is long because this file is for viewing the data in command line
    TICK = 5.0 # Update interval for the main controller loop in seconds

    def __init__(self):

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print("subscribing to topics under", topic_base_name)

        # Subscribers
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
					miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

        # Initialize a ROS node to communicate with MiRo
        rospy.init_node("sensor_suite")
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)

        # Subscribers for all sensor packages excluding the camera
        self.sub_package = rospy.Subscriber(topic_base_name + "/sensors/package",
					miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)


    def loop(self):
        """
        Main control loop
        """

        # Main control loop iteration counter
        self.counter = 0

        print("MiRo detects whether it has been touched")

        while not rospy.core.is_shutdown():

            if not self.input_package is None:
                 #aquire
                 p = self.input_package
                 self.input_package = None

                 # the sensor package includes: battery, sonar, light, and touch sensors
                 print("Update battery: ", f"{p.battery.voltage:.2f}", " V")
                 print("Body touch: ", bin(p.touch_body.data))
                 print("Head touch: ", bin(p.touch_head.data))
                 # update sonar
                 x = p.sonar.range
                 if x == 0:
                     print("Sonar: short")
                 elif x > 1.0:
                     print("Sonar: inf")
                 else:
                     print(f"Sonar: {x:.2f} m")
                 # Update light
                 x = p.light.data
                 print("Light FL: ", x[0]*100)
                 print("Light FR: ", x[1]*100)
                 print("Light RL: ", x[2]*100)
                 print("Light RR: ", x[3]*100)
                 # Display body acceleration
                 x = p.imu_body.linear_acceleration
                 print("Body accelerations")
                 print("x acceleration: ", x.x / 9.81 * 50.0)
                 print("y acceleration: ", x.y / 9.81 * 50.0)
                 print("z acceleration: ", x.z / 9.81 * 50.0)
                 l = np.sqrt(x.x*x.x + x.y*x.y + x.z*x.z)
                 print("Overall Body acceleration: ", l / 9.81 * 50.0)
                 # Display head acceleration
                 x = p.imu_head.linear_acceleration
                 print("Head accelerations")
                 print("x acceleration: ", x.x / 9.81 * 50.0)
                 print("y acceleration: ", x.y / 9.81 * 50.0)
                 print("z acceleration: ", x.z / 9.81 * 50.0)
                 l = np.sqrt(x.x*x.x + x.y*x.y + x.z*x.z)
                 print("Overall Head acceleration: ", l / 9.81 * 50.0)
                 # Display cliff sensor readings
                 x = (np.array([p.cliff.data[0], p.cliff.data[1]]) * 15).astype('int')
                 print(f"Left cliff sensor: {x[0]:.2f}")
                 print(f"Right cliff sensor: {x[1]:.2f}")
                 # Odometry readings
                 x = p.odom.twist.twist
                 print(f"linear velocity: {x.linear.x:.2f}")
                 print(f"angular velocity: {x.angular.z:.2f}")
                 # Wheel speed
                 x = p.wheel_speed_opto.data
                 print(f"Right wheel speed: {x[0]:.2f}")
                 print(f"Left wheel speed: {x[1]:.2f}")
                 # kinematic joint positions
                 x = p.kinematic_joints.position
                 print(f"Lift: {math.degrees(x[1])}")
                 print(f"Yaw: {math.degrees(x[2])}")
                 print(f"Pitch: {math.degrees(x[3])}")
                 print("\n\n\n")


            # Yield
            rospy.sleep(self.TICK)

    # Callback for touch sensor data
    def callback_package(self, msg):
         # store data from sensor package for processing
         self.input_package = msg


if __name__ == "__main__":
	main = see_touch()
	main.loop()