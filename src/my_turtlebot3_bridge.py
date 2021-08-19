#!/usr/bin/env python3

import roslibpy
import roslibpy.tf

import numpy as np
import cv2
import qimage2ndarray

import math
import sys
import base64

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

############################## global variables ##############################
form_class = uic.loadUiType(
    "/home/rirolab/catkin_ws/src/my_turtlebot3/src/my_gui.ui")[0]


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


client = roslibpy.Ros(host='192.168.0.12', port=9090)

cmdvelPub = roslibpy.Topic(
    client, '/cmd_vel', 'geometry_msgs/Twist')
cmdvelPub.advertise()

camSub = roslibpy.Topic(
    client, '/camera/color/image_raw/compressed', 'sensor_msgs/CompressedImage')

mapSub = roslibpy.Topic(
    client, '/map', 'nav_msgs/OccupancyGrid')

tfSub = roslibpy.tf.TFClient(client, '/map')
###############################################################################

##############################global functions#################################


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


def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(
        msg.info.height, msg.info.width)
    return np.ma.array(data, mask=data == -1, fill_value=-1)


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def OverlayImage(src, overlay, posx, posy):
    oh, ow, oc = overlay.shape
    sh, sw, sc = src.shape
    hh, hw = int(oh / 2), int(ow / 2)

    for x in range(ow):
        if x + posx < sw:
            for y in range(oh):
                if y + posy < sh:
                    source = src[y + posy - hh, x + posx - hw]
                    # print('source?', source)
                    over = overlay[y, x]
                    # print('over?', over)
                    if(over[3] != 0):
                        src[y + posy - hh, x + posx - hw] = over[:3]

###############################################################################


class BridgeThread(QThread):
    signalCam = pyqtSignal(np.ndarray)
    signalMap = pyqtSignal(np.ndarray)
    signalTF = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__()
        self.running = False
        self.map = np.full((384*384,), 127)
        print("Thread init")

    def run(self):
        if(self.running):
            print("Thread running")
            client.on_ready(self.startSub)
            client.run()

    def startSub(self):
        print('Im ready!!!')
        camSub.subscribe(self.camCallback)
        mapSub.subscribe(self.mapCallback)
        tfSub.subscribe('/base_link', self.tfCallback)

    def camCallback(self, msg):
        # print('Image format?', msg['format'])
        # client.get_time(lambda time: print('now?', time))
        base64_bytes = msg['data'].encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)
        np_arr = np.fromstring(image_bytes, np.uint8)
        self.signalCam.emit(np_arr)

    def mapCallback(self, msg):
        self.map = np.array(msg['data'])
        for i in range(self.map.shape[0]):
            if self.map[i] == -1:
                self.map[i] = 127
            elif self.map[i] == 0:
                self.map[i] = 255
            elif self.map[i] == 100:
                self.map[i] = 0
        self.map = self.map.reshape((384, 384))
        self.map = cv2.cvtColor(
            cv2.convertScaleAbs(self.map), cv2.COLOR_GRAY2BGR)
        self.signalMap.emit(self.map)

    def tfCallback(self, tf):
        self.signalTF.emit(tf)


class MyWindow(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.accel_but.clicked.connect(self.accelClicked)
        self.stop_but.clicked.connect(self.stopClicked)
        self.break_but.clicked.connect(self.breakClicked)
        self.left_but.clicked.connect(self.leftClicked)
        self.right_but.clicked.connect(self.rightClicked)
        self.clr_but.clicked.connect(self.clrClicked)
        self.zoom_in.clicked.connect(self.zoominClicked)
        self.zoom_out.clicked.connect(self.zoomoutClicked)
        self.exit_but.clicked.connect(QCoreApplication.instance().quit)

        self.bridge = BridgeThread(self)
        self.bridge.running = True
        self.bridge.start()
        self.bridge.signalCam.connect(self.camView)
        self.bridge.signalMap.connect(self.mapView)
        self.bridge.signalTF.connect(self.storeTF)

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.turtlebot3_model = 'waffle_pi'
        self.posx = None
        self.posy = None
        self.rot = None
        self.zoom = 1
        self.map = np.full((384, 384), 255)

    @pyqtSlot(np.ndarray)
    def camView(self, cam):
        # print("cam?", cam.shape)
        imageCV2 = cv2.imdecode(cam, cv2.IMREAD_COLOR)
        imageCV2 = cv2.cvtColor(imageCV2, cv2.COLOR_BGR2RGB)
        height, width, channel = imageCV2.shape
        # print("h,w?", height, width)
        bytesPerLine = 3 * width
        imageCV2 = self.drawDir(imageCV2)
        image = QImage(imageCV2.data, width, height,
                       bytesPerLine, QImage.Format_RGB888)
        pix = QPixmap(image).scaled(
            self.cam_label.width(), self.cam_label.height())
        self.cam_label.setPixmap(pix)

    def drawDir(self, img):
        if(self.control_linear_vel == 0 and self.control_angular_vel == 0):
            return img
        axes = (int(200 * abs(self.control_angular_vel) / WAFFLE_MAX_ANG_VEL),
                int(300 * abs(self.control_linear_vel) / WAFFLE_MAX_LIN_VEL))
        start = -90 if self.control_linear_vel > 0 else 90
        centery = 240 + 120
        if(self.control_linear_vel >= 0):
            centerx = 320 - \
                int(200 * self.control_angular_vel / WAFFLE_MAX_ANG_VEL)
            end = 0 if self.control_angular_vel > 0 else -180
        else:
            centerx = 320 + \
                int(200 * self.control_angular_vel / WAFFLE_MAX_ANG_VEL)
            end = 180 if self.control_angular_vel > 0 else 0
        color = (255, 0, 0) if self.control_linear_vel >= 0 else (0, 0, 255)
        width = 10
        ret = cv2.ellipse(img, (centerx - 100, centery),
                          axes, 0, start, end, color, width)
        ret = cv2.ellipse(img, (centerx + 100, centery),
                          axes, 0, start, end, color, width)
        return ret

    @pyqtSlot(np.ndarray)
    def mapView(self, map):
        icon = cv2.imread(
            '/home/rirolab/catkin_ws/src/my_turtlebot3/src/Turtlebot3_logo.png', cv2.IMREAD_UNCHANGED)
        icon = cv2.resize(icon, dsize=(12, 12), interpolation=cv2.INTER_AREA)
        h, w, c = icon.shape
        if (self.rot != None):
            matrix = cv2.getRotationMatrix2D(
                (int(h / 2), int(w / 2)), self.rot, 1)
            icon = cv2.warpAffine(icon, matrix, (h, w))

        mh, mw, mc = map.shape
        map = cv2.flip(cv2.rotate(map, cv2.ROTATE_90_COUNTERCLOCKWISE), 1)
        if (self.posx != None and self.posy != None):
            OverlayImage(map, icon, self.posx, self.posy)

        self.map = map
        self.drawMap()

    def drawMap(self):
        mh, mw, mc = self.map.shape
        ch, cw = mh / self.zoom, mw / self.zoom
        crop = self.map[int(mh / 2 - ch / 2):int(mh / 2 + ch / 2),
                        int(mw / 2 - cw / 2):int(mw / 2 + cw / 2)].copy()
        height, width, channel = crop.shape
        bytesPerLine = 3 * width
        image = QImage(crop.data, width, height,
                       bytesPerLine, QImage.Format_RGB888)
        pix = QPixmap(image).scaled(
            self.slam_label.width(), self.slam_label.height())
        self.slam_label.setPixmap(pix)

    @pyqtSlot(dict)
    def storeTF(self, tf):
        print("trans? at main", tf)
        trans = tf['translation']
        self.console.append('current position? (%.2f, %.2f)' %
                            (-trans['y'], trans['x']))
        quat = tf['rotation']
        rotX, rotY, rotZ = quaternion_to_euler_angle(
            quat['w'], quat['x'], quat['y'], quat['z'])
        # print('rot?', rotZ)
        self.rot = rotZ
        self.posx = int(183 - 18.3 * trans['y'])
        self.posy = int(183 - 18.3 * trans['x'])

    def zoominClicked(self):
        if self.zoom < 3:
            self.zoom += 0.5
            self.drawMap()
            self.console.append('zoom: %.1f' % self.zoom)
        else:
            self.console.append('zoom: max')

    def zoomoutClicked(self):
        if self.zoom > 1:
            self.zoom -= 0.5
            self.drawMap()
            self.console.append('zoom: %.1f' % self.zoom)
        else:
            self.console.append('zoom: min')

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
        twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

        self.control_linear_vel = makeSimpleProfile(
            self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
        twist['linear']['x'] = self.control_linear_vel
        twist['linear']['y'] = 0.0
        twist['linear']['z'] = 0.0

        self.control_angular_vel = makeSimpleProfile(
            self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
        twist['angular']['x'] = 0.0
        twist['angular']['y'] = 0.0
        twist['angular']['z'] = self.control_angular_vel

        # print("twist?", twist)
        if(client.is_connected):
            cmdvelPub.publish(roslibpy.Message(twist))
        else:
            self.console.append('connection lost...')

    def __del__(self):
        self.bridge.running = False


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
