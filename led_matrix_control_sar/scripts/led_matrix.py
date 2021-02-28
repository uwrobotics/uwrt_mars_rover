#!/usr/bin/python3

import Jetson.GPIO as GPIO
import threading
import time

''' Setting up the script:
sudo pip3 install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpi
        global continue_flashing = Falseo your_user_name
sudo cp venv/lib/pythonNN/site-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
'''

continue_flashing = False

class LEDMatrix:
    # input: pin number as labeled on the board
    def __init__(self, redPin, greenPin, bluePin):
        # pin setup
        GPIO.setup(redPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(greenPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(bluePin, GPIO.OUT, initial=GPIO.LOW)

        # pwm frequency is 50 Hz
        self.redPin = GPIO.PWM(redPin, 50)
        self.greenPin = GPIO.PWM(greenPin, 50)
        self.bluePin = GPIO.PWM(bluePin, 50)

        self.redPin.start(0)
        self.greenPin.start(0)
        self.bluePin.start(0)

        self.r = 0
        self.g = 0
        self.b = 0
        self.thread = None

    def flashing_thread(self):
        while True:
            self.set(self.r, self.g, self.b)
            time.sleep(0.5)
            self.clear()
            time.sleep(0.5)
            global continue_flashing
            if not continue_flashing:
                break

    # range is [0, 255]
    def set(self, red, green, blue):
        redDuty = red / 255.0
        greenDuty = green / 255.0
        blueDuty = blue / 255.0

        self.redPin.ChangeDutyCycle(redDuty)
        self.greenPin.ChangeDutyCycle(greenDuty)
        self.bluePin.ChangeDutyCycle(blueDuty)

    def clear(self):
        self.set(0, 0, 0)

    def flash(self, red, green, blue):
        self.r, self.g, self.b = red, green, blue

        # start flashing thread if not already flashing
        global continue_flashing
        if not continue_flashing:
            continue_flashing = True
            self.thread = threading.Thread(target=self.flashing_thread)
            self.thread.start()

    def solid(self, red, green, blue):
        # stop flashing thread
        global continue_flashing
        if continue_flashing:
            continue_flashing = False
            self.thread.join()
        
        self.r, self.g, self.b = red, green, blue
        self.set(red, green, blue)
