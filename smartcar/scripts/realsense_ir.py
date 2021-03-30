#!/usr/bin/env python3
import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import Image

def setVideoResolution(width, height):
    global fontScale
    global thickness

    PIXEL_WIDTH = width
    PIXEL_HEIGTH = height

    if PIXEL_WIDTH < 1000:
        fontScale = 1

    if PIXEL_WIDTH < 700:
        fontScale = 0.75
        thickness = 1

    return PIXEL_WIDTH, PIXEL_HEIGTH


def realsense_ir(SHOW_IMAGES, FRAMERATE, RESOLUTION_IR):

    # Defines
    font = cv.FONT_HERSHEY_SIMPLEX #for drawing on the images
    fontScale = 1.25 #for drawing on the images
    color = (0, 255, 0) #for drawing on the images
    thickness = 2 #for drawing on the images
    hole_filling = rs.hole_filling_filter()

    ## Set video resolution
    #  Example resolutions: 1024 x 768 | 640 x 480
    PIXEL_WIDTH, PIXEL_HEIGHT = setVideoResolution(width=RESOLUTION_IR[0], height=RESOLUTION_IR[1])

    # Create a pipeline
    p = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.infrared, PIXEL_WIDTH, PIXEL_HEIGHT, rs.format.y8, FRAMERATE)
    profile = p.start(config)

    # ROS defines
    pub = rospy.Publisher('realsense_ir_topic', Image, queue_size=100)

    while not rospy.is_shutdown():

        frames = p.wait_for_frames() # Check if any frame is received
        ir_frame = frames.get_infrared_frame() # Get the infrared frame

        # Validate that infrared frame is valid
        if not ir_frame:
            print("Infrared not available!")
            break

        infrared_img = np.asanyarray(ir_frame.get_data())

        if SHOW_IMAGES == True:
            # Show the image
            cv.imshow('Infrared', infrared_img)

        ir = infrared_img.ravel()
        ir = ir.tolist()

        pub.publish(height=PIXEL_HEIGHT, width=PIXEL_WIDTH, data=ir)

        #rate.sleep()

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    p.stop()

if __name__ == '__main__':
    nodeName = 'realsense_ir_camera'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName)
    # Settings
    SHOW_IMAGES = True
    FRAMERATE = 30
    RESOLUTION_IR = (640, 480)

    try:
        realsense_ir(SHOW_IMAGES, FRAMERATE, RESOLUTION_IR)
    except rospy.ROSInterruptException:
        pass