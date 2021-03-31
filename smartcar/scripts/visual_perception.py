#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
from smartcar.msg import Image_RGB, Image_Depth, Image_Infrared


def callback_all_image(Image_RGB, Image_Depth):

    b = np.frombuffer(Image_RGB.data_b, dtype=np.uint8).reshape(Image_RGB.height, Image_RGB.width)
    g = np.frombuffer(Image_RGB.data_g, dtype=np.uint8).reshape(Image_RGB.height, Image_RGB.width)
    r = np.frombuffer(Image_RGB.data_r, dtype=np.uint8).reshape(Image_RGB.height, Image_RGB.width)
    if len(Image_Depth.data_depth) > 2:
        d = np.asanyarray(Image_Depth.data_depth, dtype=np.uint16).reshape(Image_Depth.height, Image_Depth.widht)
        cv.imshow('Depth image', d)
    if len(Image_Infrared.data_ir) > 2:
        ir = np.frombuffer(Image_Infrared.data_ir, dtype=np.uint8).reshape(Image_Infrared.height, Image_Infrared.width)
        cv.imshow('Infrared image', ir)
    color_img = cv.merge([b, g, r])

    cv.imshow('Color image', color_img)
    cv.waitKey(1)

def callback_depth_image(Image_Depth):
    d = np.asanyarray(Image_Depth.data_depth, dtype=np.uint16).reshape(Image_Depth.height, Image_Depth.width)
    cv.imshow('Depth image', d)
    cv.waitKey(1)

def callback_ir_image(Image_Infrared):
    ir = np.frombuffer(Image_Infrared.data_ir, dtype=np.uint8).reshape(Image_Infrared.height, Image_Infrared.width)
    # cv.imshow('Depth image', d)
    # cv.waitKey(1)


def visual_perception():
    while True:
        # rospy.Subscriber('realsense_camera_topic', Image_RGB, Image_Depth, callback_all_image)
        rospy.Subscriber('realsense_depth_topic', Image_Depth, callback_depth_image)
        # rospy.Subscriber('realsense_ir_topic', Image_Infrared, callback_ir_image)

if __name__ == '__main__':
    nodeName = 'visual_perception'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName, anonymous=False)
    visual_perception()
    rospy.spin()