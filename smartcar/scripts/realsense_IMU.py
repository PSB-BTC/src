#!/usr/bin/env python3
import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import rospy
from smartcar.msg import Image_IMU
import matplotlib.pyplot as plt
import signal

class plot():

    def __init__(self, G):
        self.fig = plt.figure(figsize=(5, 5))
        self.G = G


    def setupPlot(self, title, subtitle, xlabel, ylabel):

        self.fig.suptitle(title, fontsize=14, fontweight='bold')
        self.subtitle = subtitle
        self.xlabel = xlabel
        self.ylabel = ylabel

        ax = self.fig.add_subplot(1,1,1)
        ax2 = self.fig.add_subplot(1,1,1)
        ax3 = self.fig.add_subplot(1,1,1)
        ax4 = self.fig.add_subplot(1,1,1)

        self.ax = ax
        self.ax2 = ax2
        self.ax3 = ax3
        self.ax4 = ax4

        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)
        self.ax.set_xlabel(self.xlabel)
        self.ax.set_ylabel(self.ylabel)
        plt.show(False)
        plt.draw()

        radius = 5
        angle = np.linspace(0, 2*np.pi, 150)
        x_circle_1 = radius * np.cos(angle)
        y_circle_1 = radius * np.sin(angle)
        self.x_circle_1 = x_circle_1
        self.y_circle_1 = y_circle_1
        self.ax3.plot(self.x_circle_1, self.y_circle_1)

        radius = 10
        x_circle_2 = radius * np.cos(angle)
        y_circle_2 = radius * np.sin(angle)
        self.x_circle_2 = x_circle_2
        self.y_circle_2 = y_circle_2

        if self.G == True:
            radius = 1
            angle = np.linspace(0, 2 * np.pi, 150)
            x_circle_1 = radius * np.cos(angle)
            y_circle_1 = radius * np.sin(angle)
            self.x_circle_1 = x_circle_1
            self.y_circle_1 = y_circle_1
            self.ax3.plot(self.x_circle_1, self.y_circle_1)

            radius = 2
            x_circle_2 = radius * np.cos(angle)
            y_circle_2 = radius * np.sin(angle)
            self.x_circle_2 = x_circle_2
            self.y_circle_2 = y_circle_2


        self.ax4.plot(x_circle_2, y_circle_2)

    def plotData(self, x, y, FREQUENCY):

        self.ax.clear()
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)

        if self.G == True:
            gain = 1/9.81

            self.ax.set_xlim(-4, 4)
            self.ax.set_ylim(-4, 4)

            x[1] = x[1]*gain
            x[2] = x[2]*gain
            y[1] = y[1]*gain
            y[2] = y[2]*gain


        self.ax.text((x[1]), y[1] - 2, 'x: {}, y: {}'.format(round(x[1], 2), round(y[1], 2)))
        self.ax.set_title(self.subtitle)
        self.ax.set_xlabel(self.xlabel)
        self.ax.set_ylabel(self.ylabel)

        self.ax.plot(x[:2], y[:2], 'o')
        self.ax2.plot(x[1:], y[1:])
        self.ax3.plot(self.x_circle_1, self.y_circle_1, color="black")
        self.ax4.plot(self.x_circle_2, self.y_circle_2, color="black")

        self.fig.canvas.draw()

        plt.pause((1 / FREQUENCY))



        plot = [self.ax, self.ax2, self.ax3, self.ax4]
        return plot


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])


def realsense_IMU(FREQUENCY, PLOT_DATA, G):
    def keyboardInterruptHandler(*args):
        print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
        print('Cleanup pipeline')
        cv.destroyAllWindows()
        p.stop()
        exit(0)
    signal.signal(signal.SIGINT, keyboardInterruptHandler)

    # Create a pipeline
    p = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, FREQUENCY)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, FREQUENCY)
    p.start(config)

    # ROS defines
    pub = rospy.Publisher('realsense_IMU_topic', Image_IMU, queue_size=100)

    if PLOT_DATA == True:
        # Create plot objects
        plot1 = plot(G)
        plot2 = plot(G)
        plot3 = plot(G)
        plot4 = plot(G)
        plot1.setupPlot(title='Acceleration X / Y axis', subtitle='Front view',xlabel= 'X-axis', ylabel='Y-axis')
        plot2.setupPlot(title='Acceleration X / Z axis', subtitle='Top view' ,xlabel= 'X-axis', ylabel='Z-axis')
        plot3.setupPlot(title='Gyro X / Y axis', subtitle='Front view', xlabel='X-axis', ylabel='Y-axis')
        plot4.setupPlot(title='Gyro X / Z axis', subtitle='Top view', xlabel='X-axis', ylabel='Z-axis')

    # Create the data lists.
    # Index 0 = 0 for reference center point
    # Index 1 = Actual position
    # Index 2 = Previous position
    x_accel = [0, 0, 0]
    y_accel = [0, 0, 0]
    z_accel = [0, 0, 0]
    x_gyro = [0, 0, 0]
    y_gyro = [0, 0, 0]
    z_gyro = [0, 0, 0]

    while not rospy.is_shutdown():

        frames = p.wait_for_frames() # Check if any frame is received
        # print(type(frames[0].as_motion_frame().get_motion_data()))
        accel = accel_data(frames[0].as_motion_frame().get_motion_data())
        gyro = gyro_data(frames[1].as_motion_frame().get_motion_data())

        # print("accelerometer| x: {} y: {} z: {}".format(accel[0],accel[1],accel[2]))
        # print("gyro         | x: {} y: {} z: {}".format(gyro[0],gyro[1],gyro[2]))

        if accel.any() or gyro.any():
            # Save the previous position for tracking visualisation
            x_accel[2] = x_accel[1]
            y_accel[2] = y_accel[1]
            z_accel[2] = z_accel[1]
            x_gyro[2] = x_gyro[1]
            y_gyro[2] = y_gyro[1]
            z_gyro[2] = z_gyro[1]

            # Get the new position
            x_accel[1] = accel[0]
            y_accel[1] = accel[1]
            z_accel[1] = accel[2]
            x_gyro[1] = gyro[0]
            y_gyro[1] = gyro[1]
            z_gyro[1] = gyro[2]

            if PLOT_DATA == True:
                plot1.plotData(x_accel, y_accel, FREQUENCY)
                plot2.plotData(x_accel, z_accel, FREQUENCY)
                plot3.plotData(x_gyro, y_gyro, FREQUENCY)
                plot4.plotData(x_gyro, z_gyro, FREQUENCY)


        pub.publish(accel=[frames[0].as_motion_frame().get_motion_data().x,
                           frames[0].as_motion_frame().get_motion_data().y,
                           frames[0].as_motion_frame().get_motion_data().z],
                    gyro= [frames[1].as_motion_frame().get_motion_data().x,
                           frames[1].as_motion_frame().get_motion_data().y,
                           frames[1].as_motion_frame().get_motion_data().z])

    cv.destroyAllWindows()
    p.stop()

if __name__ == '__main__':

    nodeName = 'realsense_IMU'
    print('ROS node "{}" has started'.format(nodeName))
    rospy.init_node(nodeName)

    # Settings
    PLOT_DATA = False
    G = False #Data unit = m/s^2. If set on True, data will be presented in G-force.
    FREQUENCY = 100 # Hz

    try:
        realsense_IMU(FREQUENCY, PLOT_DATA, G)
    except rospy.ROSInterruptException:
        pass