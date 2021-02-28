#!/usr/bin/env python
import sys
import rospy
from uwrt_mars_rover_msgs.srv import set_state
from uwrt_mars_rover_msgs.msg import NeopixelArrayMode

def set_led_matrix_client(x):
    rospy.wait_for_service('set_led_matrix')
    try:
        set_led_matrix = rospy.ServiceProxy('set_led_matrix', set_state)
        mode = NeopixelArrayMode()
        mode.value = x
        res = set_led_matrix(mode)
        return res.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    x = int(sys.argv[1])
    print("Requested mode: " + str(x))
    print("Response: " + str(set_led_matrix_client(x)))