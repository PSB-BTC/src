#!/usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
from smartcar.msg import Image_BGR
from sensor_msgs.msg import Image

def callback_all_image(Image_BGR):

    b = np.frombuffer(Image_BGR.data_b, dtype=np.uint8).reshape(Image_BGR.height, Image_BGR.width)
    g = np.frombuffer(Image_BGR.data_g, dtype=np.uint8).reshape(Image_BGR.height, Image_BGR.width)
    r = np.frombuffer(Image_BGR.data_r, dtype=np.uint8).reshape(Image_BGR.height, Image_BGR.width)
    if len(Image_BGR.data_depth) > 2:
        d = np.asanyarray(Image_BGR.data_depth, dtype=np.uint16).reshape(480, 640)
        cv.imshow('Depth image', d)
    if len(Image_BGR.data_ir) > 2:
        ir = np.frombuffer(Image_BGR.data_ir, dtype=np.uint8).reshape(480, 640)
        cv.imshow('Infrared image', ir)
    color_img = cv.merge([b, g, r])

    cv.imshow('Color image', color_img)
    cv.waitKey(1)

def callback_depth_image(Image_BGR):
    d = np.asanyarray(Image_BGR.data_depth, dtype=np.uint16).reshape(Image_BGR.height, Image_BGR.width)
    #cv.imshow('Depth image', d)
    #cv.waitKey(1)

def callback_ir_image(Image):
    ir = np.frombuffer(Image_BGR.data_ir, dtype=np.uint8).reshape(480, 640)
    # cv.imshow('Depth image', d)
    # cv.waitKey(1)


def visual_perception():
    rospy.Subscriber('realsense_camera_topic', Image_BGR, callback_all_image)
    rospy.Subscriber('realsense_depth_topic', Image_BGR, callback_depth_image)
    rospy.Subscriber('realsense_ir_topic', Image, callback_ir_image)

if __name__ == '__main__':
    nodeName = 'visual_perception'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName, anonymous=False)
    visual_perception()
    rospy.spin()