#!/usr/bin/python3
#
# Author: Bryce
# Date created: November 19, 2024
# Last modifiec: November 19, 2024
#
# Will rotate the MiRo back and forth on a set timer while active


# Misc useful python libraries
import time
import math
import os
import sys
import numpy as np

# Robot specific libraries
import rospy
import std_msgs
import geometry_msgs
import miro2 as miro

class wheel_test:

    def callback_package(self, msg):
          
          # ignore until active
          if not self.active:
                return
          
          # extract opto data
          wheel_speed = np.array(msg.wheel_speed_opto.data)

          # update controller
          self.controller.update_sensors(wheel_speed)

    def loop(self):
          
          # output
          msg_cmd_vel = geometry_msgs.msg.TwistStamped()

          TICK = 0.05
          total_time = 0.0
          cclkwise = True
          cycle_time = 1.5

          # loop
          while self.active and not rospy.core.is_shutdown():
               
               self.cmd_vel.dr = 0.0

               if cclkwise:
                    self.cmd_vel.dtheta = 1.0
               else:
                    self.cmd_vel.dtheta = -1.0

               # send velocity
               msg_cmd_vel.twist.linear.x = self.cmd_vel.dr
               msg_cmd_vel.twist.angular.z = self.cmd_vel.dtheta
               self.pub_cmd_vel.publish(msg_cmd_vel)

               # yield
               time.sleep(TICK)
               total_time += TICK
               if total_time >= cycle_time:
                    total_time = 0.0
                    cclkwise = not cclkwise

    def __init__(self):

        # state
        self.active = False
        self.controller = miro.lib.PoseController()
        self.cmd_vel = miro.lib.DeltaPose()

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribe
        topic = topic_base_name + "/sensors/package"
        print ("subscribe", topic)
        self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

        # publish
        topic = topic_base_name + "/control/cmd_vel"
        print ("publish", topic)
        self.pub_cmd_vel = rospy.Publisher(topic, geometry_msgs.msg.TwistStamped, queue_size=0)

        # publish
        topic = topic_base_name + "/control/flags"
        print ("publish", topic)
        self.pub_flags = rospy.Publisher(topic, std_msgs.msg.UInt32, queue_size=0)

        # wait for connect
        print ("\nwait for connect... ",)
        sys.stdout.flush()
        time.sleep(1)
        print ("OK")

        # send control flags
        default_flags = miro.constants.PLATFORM_D_FLAG_DISABLE_STATUS_LEDS
        msg = std_msgs.msg.UInt32()
        msg.data = default_flags
        msg.data |= miro.constants.PLATFORM_D_FLAG_PERSISTENT
        msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX
        print ("send control flags... ",)
        print (hex(msg.data),)
        self.pub_flags.publish(msg)
        print ("OK")

        # set to active
        self.active = True
    

if __name__ == "__main__":
      
      # normal sigular invocation
      rospy.init_node("wheel_test", anonymous=True)
      main = wheel_test()
      main.loop()
