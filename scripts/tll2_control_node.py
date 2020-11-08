#!/usr/bin/python

import rospy
from tll_control import control_node

if __name__ == "__main__":
    rospy.init_node("tll2_controller",anonymous=True)
    rospy.loginfo("Init node")
    control_node(tll2/control_method)
    #rospy.spin()