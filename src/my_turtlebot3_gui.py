#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
import tf2_ros

import numpy as np
import cv2
import qimage2ndarray


from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

import sys

form_class = uic.loadUiType(
    "/home/rirolab/catkin_ws/src/my_turtlebot3/src/my_gui.ui")[0]

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


def vels(target_linear_vel, target_angular_vel):
    return "currently:: linear vel %s / angular vel %s " % (target_linear_vel, target_angular_vel)


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def checkLinearLimitVelocity(vel, turtlebot3_model):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel, turtlebot3_model):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

class ImageThread(QThread):
    signalCam = pyqtSignal(np.ndarray)
    signalSlam = pyqtSignal(np.ndarray)

    def __init__(self, np_arr, parent=None):
        super().__init__()
        self.main = parent
        self.running = False

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.Subscriber("/camera/rgb/image_raw/compressed",
                         CompressedImage, self.cameraCallback, queue_size=1)
        rospy.Subscriber(
            "/map", OccupancyGrid, self.slamCallback, queue_size=1)
        self.np_arr = np_arr

    def run(self):
        if(self.running):
            trans = self.buffer.lookup_transform(
                "map", "base_link", rospy.Time())
            print("trans?", trans)
            rospy.spin()

    def cameraCallback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.signalCam.emit(np_arr)

    def slamCallback(self, ros_data):
        try:
            trans = self.buffer.lookup_transform(
                'map', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.Rate(10.0).sleep()

        print(trans.transform.translation)
        map = ros_data.data
        print(ros_data.info.resolution)
        map = np.array(map)
        # print(map.shape[0])
        for i in range(map.shape[0]):
            if map[i] == -1:
                map[i] = 127
            elif map[i] == 0:
                map[i] = 255
            elif map[i] == 100:
                map[i] = 0
        map = map.reshape((384, 384))
        map = np.flip(np.transpose(np.flip(map, axis=1)), axis=1)
        posx = int(185 - 19.2 * trans.transform.translation.x)
        posy = int(185 - 19.2 * trans.transform.translation.y)
        # posx = 185
        # posy = 185
        # print("pos?", (posx,posy))

        map[posx][posy] = 0
        map[posx + 1][posy] = 0
        map[posx][posy + 1] = 0
        map[posx + 1][posy + 1] = 0
        map[posx - 1][posy] = 0
        map[posx][posy - 1] = 0
        map[posx - 1][posy - 1] = 0
        map[posx + 1][posy - 1] = 0
        map[posx - 1][posy + 1] = 0

        self.signalSlam.emit(map)


class MyWindow(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.turtlebot3_model = rospy.get_param("model", "burger")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.accel_but.clicked.connect(self.accelClicked)
        self.stop_but.clicked.connect(self.stopClicked)
        self.break_but.clicked.connect(self.breakClicked)
        self.left_but.clicked.connect(self.leftClicked)
        self.right_but.clicked.connect(self.rightClicked)

        self.clr_but.clicked.connect(self.clrClicked)
        self.exit_but.clicked.connect(QCoreApplication.instance().quit)

        self.image_thread = ImageThread(self)
        self.image_thread.running = True
        self.image_thread.start()
        self.image_thread.signalCam.connect(self.camView)
        self.image_thread.signalSlam.connect(self.slamView)

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

    @pyqtSlot(np.ndarray)
    def camView(self, cam):
        imageCV2 = cv2.imdecode(cam, cv2.IMREAD_COLOR)
        height, width, channel = imageCV2.shape
        bytesPerLine = 3 * width
        image = QImage(imageCV2.data, width, height,
                       bytesPerLine, QImage.Format_RGB888)
        pix = QPixmap(image).scaled(
            self.cam_label.width(), self.cam_label.height())
        self.cam_label.setPixmap(pix)

    @pyqtSlot(np.ndarray)
    def slamView(self, slam):
        image = qimage2ndarray.array2qimage(slam, normalize=False)
        pix = QPixmap(image).scaled(
            self.slam_label.width(), self.slam_label.height())
        self.slam_label.setPixmap(pix)

    def clrClicked(self):
        self.console.clear()

    def accelClicked(self):
        self.target_linear_vel = checkLinearLimitVelocity(
            self.target_linear_vel + LIN_VEL_STEP_SIZE, self.turtlebot3_model)
        self.status = self.status + 1
        print(vels(self.target_linear_vel, self.target_angular_vel))
        self.console.append(
            vels(self.target_linear_vel, self.target_angular_vel))
        self.pubTeleop()

    def breakClicked(self):
        self.target_linear_vel = checkLinearLimitVelocity(
            self.target_linear_vel - LIN_VEL_STEP_SIZE, self.turtlebot3_model)
        self.status = self.status + 1
        print(vels(self.target_linear_vel, self.target_angular_vel))
        self.console.append(
            vels(self.target_linear_vel, self.target_angular_vel))
        self.pubTeleop()

    def rightClicked(self):
        self.target_angular_vel = checkAngularLimitVelocity(
            self.target_angular_vel - ANG_VEL_STEP_SIZE, self.turtlebot3_model)
        self.status = self.status + 1
        print(vels(self.target_linear_vel, self.target_angular_vel))
        self.console.append(
            vels(self.target_linear_vel, self.target_angular_vel))
        self.pubTeleop()

    def leftClicked(self):
        self.target_angular_vel = checkAngularLimitVelocity(
            self.target_angular_vel + ANG_VEL_STEP_SIZE, self.turtlebot3_model)
        self.status = self.status + 1
        print(vels(self.target_linear_vel, self.target_angular_vel))
        self.console.append(
            vels(self.target_linear_vel, self.target_angular_vel))
        self.pubTeleop()

    def stopClicked(self):
        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0
        print(vels(self.target_linear_vel, self.target_angular_vel))
        self.console.append(
            vels(self.target_linear_vel, self.target_angular_vel))
        self.pubTeleop()

    def pubTeleop(self):
        twist = Twist()

        self.control_linear_vel = makeSimpleProfile(
            self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = self.control_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        self.control_angular_vel = makeSimpleProfile(
            self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_angular_vel

        self.pub.publish(twist)

    def __del__(self):
        self.image_thread.running = False


if __name__ == '__main__':
    rospy.init_node('my_turtlebot3_gui', anonymous=True)

    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
