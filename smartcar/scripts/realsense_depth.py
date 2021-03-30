#!/usr/bin/env python3
import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import rospy
from smartcar.msg import Image_BGR
from realsense_camera import averageDistance
import time

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


def realsense_depth(DEPTH_PRE_PROCESSING, SHOW_IMAGES, FRAMERATE, RESOLUTION_DEPTH, VISUAL_PRESET):

    # Defines
    font = cv.FONT_HERSHEY_SIMPLEX #for drawing on the images
    fontScale = 1.25 #for drawing on the images
    color = (0, 255, 0) #for drawing on the images
    thickness = 2 #for drawing on the images
    hole_filling = rs.hole_filling_filter()

    ## Set video resolution
    #  Example resolutions: 1024 x 768 | 640 x 480
    PIXEL_WIDTH, PIXEL_HEIGHT = setVideoResolution(width=RESOLUTION_DEPTH[0], height=RESOLUTION_DEPTH[1])



    # Create a pipeline
    p = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, PIXEL_WIDTH, PIXEL_HEIGHT, rs.format.z16, FRAMERATE)
    profile = p.start(config)

    # ROS defines
    pub = rospy.Publisher('realsense_depth_topic', Image_BGR, queue_size=100)

    # Collect depth scaling data from depth sensor
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    visual_preset_dict = {'custom':0.0,
                          'default':1.0,
                          'no_ambient_light':2.0,
                          'low_ambient_light':3.0,
                          'max_range':4.0,
                          'short_range':5.0}


    set_preset = visual_preset_dict[VISUAL_PRESET]

    depth_sensor.set_option(rs.option.visual_preset, set_preset)
    time.sleep(1)

    print('Visual preset set: {}. | {}'.format(depth_sensor.get_option(rs.option.visual_preset), VISUAL_PRESET))

    laser_power = depth_sensor.get_option(rs.option.laser_power)
    laser_range = depth_sensor.get_option_range(rs.option.laser_power)
    print("laser power range = ", laser_range.min, "~", laser_range.max)
    set_laser = 0
    if laser_power + 10 > laser_range.max:
        set_laser = laser_range.max
    else:
        set_laser = laser_power + 10

    depth_sensor.set_option(rs.option.laser_power, 100)
    time.sleep(1)
    print('laserpower =',depth_sensor.get_option(rs.option.laser_power))






    # preset_range = depth_sensor.get_option_range(rs.option.hardware_preset)

    # print('preset range:'+str(preset_range))
    # for i in range(int(preset_range.max)):
    #     hardware_preset = depth_sensor.get_option_value_description(rs.option.hardware_preset, i)
    #     print('{}. | hardware_preset = {}'.format(i, hardware_preset))
    #     if hardware_preset == "Max Range":
    #         depth_sensor.set_option(rs.option.hardware_preset, i)
    #         set_preset = hardware_preset
    #         print('Visual preset set: {}. | {}'.format(depth_sensor.get_option(rs.option.hardware_preset), set_preset))
    #         break




    while not rospy.is_shutdown():

        frames = p.wait_for_frames() # Check if any frame is received
        depth_frame = frames.get_depth_frame() # Get the depth frame

        # Validate that depth frame is valid
        if not depth_frame:
            print("Color or Depth not available!")
            break

        # Apply depth filter
        if DEPTH_PRE_PROCESSING == True:
            depth_frame = hole_filling.process(depth_frame)

        # Define image center point
        image_origin = int((PIXEL_WIDTH / 2)), int(PIXEL_HEIGHT / 2)

        # Colorize depth image and process to numpy array
        c = rs.colorizer()
        colorized_depth = np.asanyarray(c.colorize(depth_frame).get_data())
        uncolorized_depth = np.asanyarray(depth_frame.get_data())

        # Get distance information of centre pixel
        depth_pixel = uncolorized_depth[image_origin[0], image_origin[1]].astype(float)
        distance = round(depth_pixel*depth_scale , 2)

        # Box size (ROI) around image origin
        box_width = 2
        box_height = 2

        avgDist = averageDistance(uncolorized_depth,
                                  start_pixel_x=(image_origin[0] - int((box_width / 2))),
                                  # Define coordinate of ROI box X-axis (left boundary)
                                  size_x=(image_origin[0] + int((box_width / 2))),
                                  # Define coordinate of ROI box X-axis (right boundary)
                                  start_pixel_y=(image_origin[1] - int((box_height / 2))),
                                  # Define coordinate of ROI box Y-axis (upper boundary)
                                  size_y=(image_origin[1] + int((box_height / 2))),
                                  depth_scale=depth_scale)
                                  # Define coordinate of ROI box Y-axis (lower boundary)

        if SHOW_IMAGES == True:
            # Draw on the image
            cv.putText(colorized_depth, "Average distance: " + str(distance) + ' m', (300, 300), font, fontScale, color,
                       thickness)
            leftTop = int(PIXEL_WIDTH / 2 - box_width / 2), int(PIXEL_HEIGHT / 2 - box_height / 2)
            rightBottom = int(PIXEL_WIDTH / 2 + box_width / 2), int(PIXEL_HEIGHT / 2 + box_height / 2)
            cv.rectangle(colorized_depth, leftTop, rightBottom, color, thickness)

            # Show the image
            cv.imshow('Depth', colorized_depth)

        d = uncolorized_depth.ravel()
        d = d.tolist()

        pub.publish(height=PIXEL_HEIGHT, width=PIXEL_WIDTH, data_depth=d)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    p.stop()

if __name__ == '__main__':
    nodeName = 'realsense_depth_camera'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName)
    # Settings
    DEPTH_PRE_PROCESSING = False
    SHOW_IMAGES = True
    FRAMERATE = 30 # Only option for depth image
    RESOLUTION_DEPTH = (1024, 768)  # Example resolutions depth image (RS L515): 640 x 480 | 1024 x 768
    VISUAL_PRESET = 'short_range' # Options are: custom, default, no_ambient_light, low_ambient_light, max_range, short_range

    try:
        realsense_depth(DEPTH_PRE_PROCESSING, SHOW_IMAGES, FRAMERATE, RESOLUTION_DEPTH, VISUAL_PRESET)
    except rospy.ROSInterruptException:
        pass