#!/usr/bin/env python
import Jetson.GPIO as GPIO
from led_matrix import LEDMatrix

from uwrt_mars_rover_msgs.srv import set_state
from uwrt_mars_rover_msgs.msg import NeopixelArrayMode
import rospy

# Jetson Xavier Pinout: https://www.jetsonhacks.com/nvidia-jetson-agx-xavier-gpio-header-pinout/
R_PWM1 = 13
G_PWM2 = 15
B_PWM3 = 18

matrix = None

def set_led_matrix_handler(req):
    retval = False

    if req.requested_mode.value == NeopixelArrayMode.AUTONOMOUS_OPERATION:
        matrix.solid(255, 0, 0) # solid red
        print('LED Matrix set to AUTONOMOUS_OPERATION')
        retval = True
    
    elif req.requested_mode.value == NeopixelArrayMode.TELEOPERATED_OPERATION:
        matrix.solid(0, 0, 255) # solid blue
        print('LED Matrix set to TELEOPERATED_OPERATION')
        retval = True

    elif req.requested_mode.value == NeopixelArrayMode.AUTONOMOUS_GOAL_REACHED:
        matrix.flash(0, 255, 0) # flash green
        print('LED Matrix set to AUTONOMOUS_GOAL_REACHED')
        retval = True
    
    elif req.requested_mode.value == NeopixelArrayMode.OFF:
        matrix.solid(0, 0, 0) # clear
        print('LED Matrix set to OFF')
        retval = True
    
    return retval

def set_led_matrix_server():
    rospy.init_node('set_led_matrix_server')
    s = rospy.Service('set_led_matrix', set_state, set_led_matrix_handler)
    print("Ready to set LED matrix")
    rospy.spin()


if __name__ == "__main__":
    GPIO.setmode(GPIO.BOARD)
    matrix = LEDMatrix(R_PWM1, G_PWM2, B_PWM3)

    set_led_matrix_server()
