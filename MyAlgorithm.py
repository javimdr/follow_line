#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np
import threading
import time
from datetime import datetime
import cv2
from code.controls import *
from code.control_pd import *


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, cameraL, cameraR, motors):
        self.cameraL = cameraL
        self.cameraR = cameraR
        self.motors = motors
        self.imageRight = None
        self.imageLeft = None
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()

        """
        self.references = {'x':  330.0,
                           'ux': 15,
                           'm': -21.5,
                           'um': 5,
                           'last_state': "recta",
                           'error': 0}
        """

        self.car_control = controller()
        # camera out: 400 x 660
        self.last_x = 0.5
        self.last_w = 0
        self.last_v = 9 # 0
        self.last_e = 'recta'
        self.test_to_run = 9 #0

        self.analizator = ImageAnalizator()
        self.road = RoadAnalizator(-87.34)
        self.control = pd_controller(0.5)

        threading.Thread.__init__(self, args=self.stop_event)



    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage

    def run (self):
        while (not self.kill_event.is_set()):
            start_time = datetime.now()
            if not self.stop_event.is_set():
                self.execute()
            finish_Time = datetime.now()
            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print(ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    def execute(self):
        #GETTING THE IMAGES (480, 680)
        imageLeft = self.cameraL.getImage()
        imageRight = self.cameraR.getImage()
        # Add your code here
        #print ("Running")

        img = self.analizator.color_filter(imageLeft.data)
        rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        empty = np.ones(rgb.shape, np.uint8)*255
        v, w = self.car_control.execute(img, rgb, empty)

        self.motors.sendV(v)
        self.motors.sendW(w)
        self.setLeftImageFiltered(rgb)
        self.setRightImageFiltered(empty)

