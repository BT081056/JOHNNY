# -*- coding: utf-8 -*-
"""
Created on Tue Aug 18 15:59:10 2020

@author: 2007020
"""
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import RPi.GPIO as GPIO
import cv2
import matplotlib.pyplot as plt
import sys
import pandas as pd
import numpy as np
from ui_MainWindow import Ui_MainWindow
import threading
import time
import datetime
import os
# 影片檔案
videoFile = 0
input_pin = 18  # BCM pin 18, BOARD pin 12
LED_G=17         # BCM pin 17, BOARD pin 11
LED_R=27        # BCM pin 27, BOARD pin 13
# 開啟影片檔
GPIO.setmode(GPIO.BCM)  # 設置為BCM模式（您也可以自行改為BOARD模式）
GPIO.setup(input_pin, GPIO.IN)  # 設置輸入的腳位為input_pin
GPIO.setup(LED_R, GPIO.OUT)  # 設置輸入的腳位為input_pin
GPIO.setup(LED_G, GPIO.OUT)  # 設置輸入的腳位為input_pin
GPIO.output(LED_R,GPIO.LOW)
GPIO.output(LED_G,GPIO.LOW)
n = 0
spec = 890

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.n = 1
        # 執行緒自定義訊號連線的槽函式
        self.work = WorkThread()
        self.work.start()
        self.work.changePixmap.connect(self.img_output.setPixmap)
        
        self.start_btn.clicked.connect(self.execute)
        self.close_btn.clicked.connect(QCoreApplication.instance().quit)

    def execute(self):
        DT = datetime.datetime.today().strftime('%Y-%m-%d_%H%m%S')
        SaveFile = '/home/pi/Desktop/GETCAMERA/IMAGE'
        Resultf = os.path.join(SaveFile,DT+".jpg")
        cv2.imwrite(Resultf,self.work.getframe())
        self.img_output_2.setPixmap(QPixmap(Resultf))
        self.img_output_2.setScaledContents(True)

class WorkThread(QThread):
    # 自定義訊號物件。引數str就代表這個訊號可以傳一個字串
    changePixmap = pyqtSignal(QPixmap)
    
    def __int__(self):
        # 初始化函式
        super(WorkThread, self).__init__()
        
    def getframe(self):
        return self.frame


    def run(self):
        #重寫執行緒執行的run函式
        #觸發自定義訊號
        cap = cv2.VideoCapture(videoFile)

        while True:
            ret, self.frame = cap.read()
            rgbImage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            convertToQtFormat = QImage(rgbImage.data, rgbImage.shape[1], rgbImage.shape[0], QImage.Format_RGB888)
            convertToQtFormat = QPixmap.fromImage(convertToQtFormat)
            p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
            self.changePixmap.emit(p)
     
        
    
if __name__=='__main__':
    app=QApplication(sys.argv)
    window=MainWindow()
    window.show()
    sys.exit(app.exec())
