#!/usr/bin/env python3
import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import time
import rospy
from smartcar.msg import Image_BGR


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

def averageDistance(depth_image, start_pixel_x, size_x, start_pixel_y, size_y, depth_scale):

    ROI_Y = list(range(start_pixel_y, size_y))
    ROI_X = list(range(start_pixel_x, size_x))

    i = 0
    j = 0
    averageList = []
    for y in ROI_Y:
        for x in ROI_X:
            depth_pixel = (depth_image[(start_pixel_x+i),(start_pixel_y+j) ].astype(int))*depth_scale
            averageList.append(depth_pixel)
            #print('Coordinate: {}, {} || pixel value: {}'.format((ROI_X[i]), (ROI_Y[j]), depth_pixel))
            i += 1

        i = 0
        j += 1

    avgDist = round(sum(averageList)/len(averageList), 2)
    return avgDist

def realsense_rgb(DEPTH_ENABLED, DEPTH_PRE_PROCESSING, SHOW_IMAGES, FRAMERATE, RESOLUTION_COLOR, RESOLUTION_DEPTH, RESOLUTION_IR, IR_ENABLED, VISUAL_PRESET):

    # Defines
    font =          cv.FONT_HERSHEY_SIMPLEX # for drawing on the images
    fontScale =     1.25 # for drawing on the images
    color =         (0, 255, 0) # for drawing on the images
    thickness =     2 # for drawing on the images
    hole_filling =  rs.hole_filling_filter()




    # Set video resolution
    PIXEL_WIDTH, PIXEL_HEIGHT =                 setVideoResolution(width=RESOLUTION_COLOR[0],
                                                                   height=RESOLUTION_COLOR[1])
    PIXEL_WIDTH_DEPTH, PIXEL_HEIGHT_DEPTH =     setVideoResolution(width=RESOLUTION_DEPTH[0],
                                                                   height=RESOLUTION_DEPTH[1])
    PIXEL_WIDTH_IR, PIXEL_HEIGHT_IR =           setVideoResolution(width=RESOLUTION_IR[0],
                                                                   height=RESOLUTION_IR[1])
    if DEPTH_ENABLED and IR_ENABLED == True:
        PIXEL_WIDTH_DEPTH, PIXEL_HEIGHT_DEPTH = setVideoResolution(width=RESOLUTION_IR[0],
                                                                   height=RESOLUTION_IR[1])

    if (DEPTH_ENABLED or IR_ENABLED == True) or PIXEL_WIDTH == 1920:
        FRAMERATE = 30

    # Allign frames. NOTE: Possible performance issues when aligning
    # align_to = rs.stream.color
    # align = rs.align(align_to)

    # Create a pipeline
    p =      rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, PIXEL_WIDTH, PIXEL_HEIGHT, rs.format.rgb8, FRAMERATE)
    if DEPTH_ENABLED == True:
        config.enable_stream(rs.stream.depth, PIXEL_WIDTH_DEPTH, PIXEL_HEIGHT_DEPTH, rs.format.z16, FRAMERATE)
    if IR_ENABLED == True:
        config.enable_stream(rs.stream.infrared, PIXEL_WIDTH_IR, PIXEL_HEIGHT_IR, rs.format.y8, FRAMERATE)
    profile = p.start(config)

    # ROS defines
    pub1 = rospy.Publisher('realsense_camera_topic', Image_BGR, queue_size=100)

    # Collect depth scaling information from pipeline
    depth_sensor = profile.get_device().first_depth_sensor() # Collect depth scaling data from depth sensor
    depth_scale = depth_sensor.get_depth_scale()

    if DEPTH_ENABLED == True:

        visual_preset_dict = {'custom': 0.0,
                              'default': 1.0,
                              'no_ambient_light': 2.0,
                              'low_ambient_light': 3.0,
                              'max_range': 4.0,
                              'short_range': 5.0}

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

        depth_sensor.set_option(rs.option.laser_power, set_laser)
        time.sleep(1)
        print('laserpower =', depth_sensor.get_option(rs.option.laser_power))

    while not rospy.is_shutdown():

        start = time.time() # For FPS calculation

        frames = p.wait_for_frames() # Check if any frame is received
        color_frame = frames.get_color_frame() # Get the color frame

        # Validate that color frame is valid
        if not color_frame:
            print("Color frame not available!")
            break

        # Process color image to numpy array
        color_img = np.asanyarray(color_frame.get_data())
        color_img = cv.cvtColor(color_img, cv.COLOR_BGR2RGB)
        b, g, r = cv.split(color_img)

        if DEPTH_ENABLED == True:

            depth_frame = frames.get_depth_frame()

            # Define image center point
            image_origin = int((depth_frame.get_width() / 2)), int(depth_frame.get_height() / 2)

            # Apply depth filter
            if DEPTH_PRE_PROCESSING == True:
                depth_frame = hole_filling.process(depth_frame)

            # Colorize depth image and process to numpy array
            c = rs.colorizer()
            colorized_depth = np.asanyarray(c.colorize(depth_frame).get_data())
            uncolorized_depth = np.asanyarray(depth_frame.get_data())

            # Get distance information of centre pixel
            depth_pixel = uncolorized_depth[image_origin[0], image_origin[1]].astype(float)
            distance = round(depth_pixel * depth_scale, 2)

            # Box size (ROI) around image origin
            box_width  = 50
            box_height = 50

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

            # Show the depth image
            if SHOW_IMAGES == True:
                # Draw on the image
                cv.putText(color_img, "Distance: " + str(distance) + ' m', (300, 300), font, fontScale, color, thickness)
                leftTop = int(PIXEL_WIDTH / 2 - box_width / 2), int(PIXEL_HEIGHT / 2 - box_height / 2)
                rightBottom = int(PIXEL_WIDTH / 2 + box_width / 2), int(PIXEL_HEIGHT / 2 + box_height / 2)
                cv.rectangle(color_img, leftTop, rightBottom, color, thickness)

                cv.imshow('Depth', colorized_depth)

            d = uncolorized_depth.ravel()
            d = d.tolist()

        if IR_ENABLED == True:

            ir_frame = frames.get_infrared_frame()
            infrared_img = np.asanyarray(ir_frame.get_data())

            if SHOW_IMAGES == True: # Show the ir image

                cv.imshow('Infrared', infrared_img)

            ir = infrared_img.ravel()
            ir = ir.tolist()

        if DEPTH_ENABLED == False:
            d = [0]

        if IR_ENABLED == False:
            ir = [0]

        ## Prepare RGB image for publishing on ROS topic
        # reshape 2D matrix to 1D array
        b = b.ravel()
        g = g.ravel()
        r = r.ravel()

        # transform numpy array to python list
        b = b.tolist()
        g = g.tolist()
        r = r.tolist()

        pub1.publish(height=PIXEL_HEIGHT, width=PIXEL_WIDTH, data_b=b, data_g=g, data_r=r, data_depth=d, data_ir= ir)

        end = time.time()
        FPS = round(1/(end-start), 1)

        # Show the color image
        if SHOW_IMAGES == True:
            # Draw on the image
            cv.putText(color_img, "FPS: " + str(FPS), (50, 50), font, fontScale, color, thickness)

            # Show the image
            cv.imshow('Color', color_img)

        #rate.sleep()
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    p.stop()

if __name__ == '__main__':
    nodeName = 'realsense_camera'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName)

    # Settings

    DEPTH_ENABLED = False
    DEPTH_PRE_PROCESSING = True
    VISUAL_PRESET = 'max_range' # Options are: custom, default, no_ambient_light, low_ambient_light, max_range, short_range
    IR_ENABLED = False

    SHOW_IMAGES = True
    FRAMERATE = 30


    RESOLUTION_COLOR = (640, 480) #  Example resolutions color image (RS L515): 640 x 480 | 1280 x 720 | 1920 x 1080
    RESOLUTION_DEPTH = (640, 480) #  Example resolutions depth image (RS L515): 640 x 480 | 1024 x 768
    RESOLUTION_IR    = (640, 480)

    try:
        realsense_rgb(DEPTH_ENABLED, DEPTH_PRE_PROCESSING, SHOW_IMAGES, FRAMERATE, RESOLUTION_COLOR, RESOLUTION_DEPTH, RESOLUTION_IR, IR_ENABLED, VISUAL_PRESET)

    except rospy.ROSInterruptException:
        pass