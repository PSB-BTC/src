#!/usr/bin/env python3

import rospy
import Jetson.GPIO as GPIO
import time
from sensor_msgs.msg import Range
import signal


# Source datasheet download: https://www.kiwi-electronics.nl/ultrasonic-sensor-hc-sr04&search=ultrasoon%20sensor%20hc-&description=true
# File: HCSR04

def keyboardInterruptHandler(signal, frame):
    print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
    print('Cleanup GPIO')
    GPIO.cleanup()
    exit(0)

def delayMicroSecond(us):
    s = us * 0.000006
    time.sleep(s)

def delayMilliSecond(ms):
    s = ms * 0.001
    time.sleep(s)

def ultrasonicJetson():

    pub = rospy.Publisher('ultrasonic_sensor_1', Range, queue_size=10)

    # GPIO.setwarnings(False)

    MAX_DIST = 23200

    GPIO.setmode(GPIO.CVM)
    trigger_channel = 'GPIO01'
    echo_channel = 'GPIO11'
    # trigger_channel = 'GPIO13'
    # echo_channel = 'GPIO07'

    GPIO.setup(trigger_channel, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(echo_channel, GPIO.IN)

    t1 = 0
    t2 = 0
    pulse_width = 0
    cm = 0
    loop = True
    timeout = 1
    signal.signal(signal.SIGINT, keyboardInterruptHandler)

    while (loop == True):

        # Send trigger signal
        GPIO.output(trigger_channel, GPIO.HIGH)
        delayMicroSecond(10)
        GPIO.output(trigger_channel, GPIO.LOW)

        to1 = time.time()

        # Wait for receivement of echo
        while (GPIO.input(echo_channel) == 0):

            to2 = time.time()
            if to2 - to1 > timeout:
                # timeout
                break

            continue

        t1 = time.time() * 1000000
        to1 = time.time()

        # Wait for end of echo signal
        while (GPIO.input(echo_channel) == 1):

            to2 = time.time()
            if to2 - to1 > timeout:
                # timout
                break

            continue

        t2 = time.time() * 1000000
        pulse_width = t2 - t1

        cm = round((pulse_width / 58.0), 2)

        # if (pulse_width > MAX_DIST):
        #     print('Out of range')
        #
        # else:
        #     print('Distance is: {} cm'.format(cm))

        # Wait before next measurement
        delayMilliSecond(60)

        pub.publish(radiation_type=0, min_range=2, max_range=400, range=cm)



if __name__ == '__main__':
    nodeName = 'ultrasonic_1_front'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName, anonymous=False)
    ultrasonicJetson()







