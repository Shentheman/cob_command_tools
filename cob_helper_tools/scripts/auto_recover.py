#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) Felix Messmer \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n
#
#   All rights reserved. \n\n
#
#################################################################
#
# \note
#   Repository name: cob_command_tools
# \note
#   ROS package name: cob_helper_tools
#
# \author
#   Author: Felix Messmer
#
# \date Date of creation: January 2017
#
# \brief
#   A script to automatically recover hardware after e-stop and HW failure
#
#################################################################

import copy
import rospy

from cob_msgs.msg import EmergencyStopState
from diagnostic_msgs.msg import DiagnosticArray

from simple_script_server import *
sss = simple_script_server()

class AutoRecover():

  def __init__(self):
    now = rospy.Time.now()
    self.em_state = 0
    self.components = rospy.get_param('~components', {})
    self.components_recover_time = {}
    for component in self.components.keys():
      self.components_recover_time[component] = now
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.em_cb, queue_size=1)
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_cb, queue_size=1)

  # auto recover based on diagnostics
  def em_cb(self, msg):
    if msg.emergency_state == 0 and self.em_state != 0:
      rospy.loginfo("auto_recover from emergency state")
      self.recover(self.components.keys())
    self.em_state = copy.deepcopy(msg.emergency_state)

  def recover(self, components):
    for component in components:
      handle = sss.recover(component)
      if not (handle.get_error_code() == 0):
        rospy.logerr("[auto_recover]: Could not recover %s", component)
      else:
        rospy.loginfo("[auto_recover]: Component %s recovered successfully", component)
        self.components_recover_time[component] = rospy.Time.now()

  # auto recover based on diagnostics
  def diagnostics_cb(self, msg):
    for status in msg.status:
      for component in self.components.keys():
        if status.name.startswith(self.components[component]) and status.level > 0 and self.em_state == 0 and (rospy.Time.now() - self.components_recover_time[component] > rospy.Duration(10)):
          rospy.loginfo("auto_recover from diagnostic failure")
          self.recover([component])

if __name__ == "__main__":
  rospy.init_node("auto_recover")
  AR = AutoRecover()
  rospy.loginfo("auto recover running")
  rospy.spin()
