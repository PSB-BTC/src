#!/usr/bin/env python3
import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import time
import rospy
from smartcar.msg import Image_RGB, Image_Depth, Image_Infrared, Realsense_Data
import signal


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

def realsense_rgb(DEPTH_PRE_PROCESSING, ALLIGN_FRAMES, SHOW_IMAGES, SEND_IMAGES, FRAMERATE, RESOLUTION_RGB, RESOLUTION_DEPTH_IR, VISUAL_PRESET, COLOR, GRAY):
    def keyboardInterruptHandler(*args):
        print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
        print('Cleanup pipeline')
        cv.destroyAllWindows()
        p.stop()
        exit(0)
    signal.signal(signal.SIGINT, keyboardInterruptHandler)

    # Defines
    font =          cv.FONT_HERSHEY_SIMPLEX # for drawing on the images
    fontScale =     1.25 # for drawing on the images
    color =         (0, 255, 0) # for drawing on the images
    thickness =     2 # for drawing on the images
    hole_filling =  rs.hole_filling_filter()

    # Set video resolution
    PIXEL_WIDTH, PIXEL_HEIGHT =                 setVideoResolution(width=RESOLUTION_RGB[0],
                                                                   height=RESOLUTION_RGB[1])
    PIXEL_WIDTH_DEPTH, PIXEL_HEIGHT_DEPTH =     setVideoResolution(width=RESOLUTION_DEPTH_IR[0],
                                                                   height=RESOLUTION_DEPTH_IR[1])
    PIXEL_WIDTH_IR, PIXEL_HEIGHT_IR =           setVideoResolution(width=RESOLUTION_DEPTH_IR[0],
                                                                   height=RESOLUTION_DEPTH_IR[1])

    if (DEPTH_ENABLED or IR_ENABLED) or PIXEL_WIDTH == 1920:
        FRAMERATE = 30

    if not RGB_ENABLED:
        COLOR = False
        GRAY = False

    # Create a pipeline

    p =      rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, PIXEL_WIDTH, PIXEL_HEIGHT, rs.format.rgb8, FRAMERATE)
    if DEPTH_ENABLED:
        config.enable_stream(rs.stream.depth, PIXEL_WIDTH_DEPTH, PIXEL_HEIGHT_DEPTH, rs.format.z16, FRAMERATE)
    if IR_ENABLED:
        config.enable_stream(rs.stream.infrared, PIXEL_WIDTH_IR, PIXEL_HEIGHT_IR, rs.format.y8, FRAMERATE)

    profile = p.start(config)

    if SEND_IMAGES:
        # ROS defines
        pub_RGB = rospy.Publisher('realsense_rgb_video', Image_RGB, queue_size=100)
        pub_DEPTH = rospy.Publisher('realsense_depth_video', Image_Depth, queue_size=100)
        pub_IR = rospy.Publisher('realsense_ir_video', Image_Infrared, queue_size=100)

    pub = rospy.Publisher('realsense_data', Realsense_Data, queue_size=100)

    # Collect depth scaling information from pipeline
    depth_sensor = profile.get_device().first_depth_sensor() # Collect depth scaling data from depth sensor
    depth_scale = depth_sensor.get_depth_scale()

    if DEPTH_ENABLED:

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

    time.sleep(1)
    while not rospy.is_shutdown():

        # start = time.time() # For FPS calculation

        frames = p.wait_for_frames() # Check if any frame is received

        if RGB_ENABLED:

            color_frame = frames.get_color_frame() # Get the color frame

            # Validate that color frame is valid
            if not color_frame:
                print("Color frame not available!")
                break

            # Process color image to numpy array
            color_img = np.asanyarray(color_frame.get_data())
            color_img = cv.cvtColor(color_img, cv.COLOR_BGR2RGB)
            gray_img = cv.cvtColor(color_img, cv.COLOR_BGR2GRAY)

            if SEND_IMAGES:
                if COLOR:
                    b, g, r = cv.split(color_img)

                    ## Prepare RGB image for publishing on ROS topic
                    # reshape 2D matrix to 1D array
                    b = b.ravel()
                    g = g.ravel()
                    r = r.ravel()

                    # transform numpy array to python list
                    b = b.tolist()
                    g = g.tolist()
                    r = r.tolist()

                if GRAY:
                    gray = gray_img.ravel()
                    gray = gray.tolist()

        if DEPTH_ENABLED:

            depth_frame = frames.get_depth_frame()
            x, y = int((depth_frame.get_width() / 2)), int(depth_frame.get_height() / 2) #define image origin

            # Apply depth filter
            if DEPTH_PRE_PROCESSING:
                depth_frame = hole_filling.process(depth_frame)

            c = rs.colorizer()

            if ALLIGN_FRAMES:
                # Allign frames. NOTE: Possible performance issues when aligning
                align = rs.align(rs.stream.color)
                frames = align.process(frames)

                alligned_depth_frame = frames.get_depth_frame()
                colorized_depth = np.asanyarray(c.colorize(alligned_depth_frame).get_data())

                depth = np.asanyarray(alligned_depth_frame.get_data())

                # update image origin
                x, y = int((alligned_depth_frame.get_width() / 2)), int(alligned_depth_frame.get_height() / 2)

                uncolorized_depth = np.asanyarray(alligned_depth_frame.get_data())
            else:

                colorized_depth = np.asanyarray(c.colorize(depth_frame).get_data())
                uncolorized_depth = np.asanyarray(depth_frame.get_data())
                depth = uncolorized_depth


            # Box size (ROI) around image origin
            box_width  = 50
            box_height = 50

            # avgDist = averageDistance(uncolorized_depth,
            #                           start_pixel_x=(image_origin[0] - int((box_width / 2))),
            #                           # Define coordinate of ROI box X-axis (left boundary)
            #                           size_x=(image_origin[0] + int((box_width / 2))),
            #                           # Define coordinate of ROI box X-axis (right boundary)
            #                           start_pixel_y=(image_origin[1] - int((box_height / 2))),
            #                           # Define coordinate of ROI box Y-axis (upper boundary)
            #                           size_y=(image_origin[1] + int((box_height / 2))),
            #                           depth_scale=depth_scale)
            #                           # Define coordinate of ROI box Y-axis (lower boundary)

            x, y ,w, h = int(x-(box_width/2)), int(y-(box_height/2)), box_width, box_height
            depth = depth[x:x+w, y:y+h].astype(float)
            depth_crop = depth.copy()
            depth_res = depth_crop[depth_crop != 0]
            depth_res = depth_res * depth_scale

            distance = 0.0
            if depth_res.size != 0:
                distance = round(min(depth_res), 2)

            # Show the depth image
            if SHOW_IMAGES:
                cv.imshow('Depth', colorized_depth)

            if SEND_IMAGES:
                d = uncolorized_depth.ravel()
                # d = d.tolist()


        if IR_ENABLED:

            ir_frame = frames.get_infrared_frame()
            infrared_img = np.asanyarray(ir_frame.get_data())

            if SHOW_IMAGES: # Show the ir image

                cv.imshow('Infrared', infrared_img)

            if SEND_IMAGES:
                ir = infrared_img.ravel()
                ir = ir.tolist()

        if not DEPTH_ENABLED:
            d = [0]
            distance = 0

        if not IR_ENABLED:
            ir = [0]

        if not COLOR:
            b = [0]
            g = [0]
            r = [0]

        if not GRAY:
            gray = [0]

        if SEND_IMAGES:
            pub_RGB.publish(height=PIXEL_HEIGHT, width=PIXEL_WIDTH, data_r=r, data_g=g, data_b=b, data_grey=gray)
            pub_DEPTH.publish(height=PIXEL_HEIGHT_DEPTH, width=PIXEL_WIDTH_DEPTH, data_depth=d)
            pub_IR.publish(height=PIXEL_HEIGHT_IR, width=PIXEL_WIDTH_IR, data_ir=ir)

        pub.publish(distance=distance)

        # end = time.time()
        # FPS = round(1/(end-start), 1)
        # print("FPS: {}".format(FPS))

        # Show the color image
        if SHOW_IMAGES == True and RGB_ENABLED == True:

            if COLOR:
                # cv.putText(color_img, "FPS: " + str(FPS), (50, 50), font, fontScale, color, thickness)
                if DEPTH_ENABLED:
                    cv.putText(color_img, "Distance: " + str(distance) + ' m', (300, 300), font, fontScale, color, thickness)
                cv.imshow('Color', color_img)

            if GRAY:
                # cv.putText(gray_img, "FPS: " + str(FPS), (50, 50), font, fontScale, color, thickness)
                if DEPTH_ENABLED:
                    cv.putText(gray_img, "Distance: " + str(distance) + ' m', (300, 300), font, fontScale, color, thickness)
                cv.imshow('Grayscale', gray_img)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    p.stop()


if __name__ == '__main__':
    nodeName = 'realsense_camera'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName)

    # Settings
    global RGB_ENABLED
    global DEPTH_ENABLED
    global IR_ENABLED

    RGB_ENABLED = True
    COLOR = True
    GRAY = True

    DEPTH_ENABLED = True
    DEPTH_PRE_PROCESSING = False
    ALLIGN_FRAMES = False
    VISUAL_PRESET = 'short_range' # Options are: custom, default, no_ambient_light, low_ambient_light, max_range, short_range
    IR_ENABLED = True

    SHOW_IMAGES = False
    SEND_IMAGES = True
    FRAMERATE = 30


    RESOLUTION_RGB = (1280, 720) #  Example resolutions color image (RS L515): 640 x 480 | 1280 x 720 | 1920 x 1080
    RESOLUTION_DEPTH_IR = (1024, 768) #  Example resolutions depth image (RS L515): 640 x 480 | 1024 x 768

    try:
        realsense_rgb(DEPTH_PRE_PROCESSING, ALLIGN_FRAMES, SHOW_IMAGES, SEND_IMAGES, FRAMERATE, RESOLUTION_RGB, RESOLUTION_DEPTH_IR, VISUAL_PRESET, COLOR, GRAY)

    except rospy.ROSInterruptException:
        pass