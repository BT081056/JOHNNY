# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1200, 640)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.img_output = QtWidgets.QLabel(self.centralwidget)
        self.img_output.setGeometry(QtCore.QRect(20, 10, 600, 450))
        self.img_output.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.img_output.setFrameShadow(QtWidgets.QFrame.Raised)
        self.img_output.setObjectName("img_output")
        self.img_output_2 = QtWidgets.QLabel(self.centralwidget)
        self.img_output_2.setGeometry(QtCore.QRect(620, 10, 600, 450))
        self.img_output_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.img_output_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.img_output_2.setObjectName("img_output_2")
        self.start_btn = QtWidgets.QPushButton(self.centralwidget)
        self.start_btn.setGeometry(QtCore.QRect(20, 500, 200, 100))
        font = QtGui.QFont()
        font.setFamily("微軟正黑體")
        font.setPointSize(18)
        font.setBold(False)
        font.setWeight(50)
        self.start_btn.setFont(font)
        self.start_btn.setObjectName("Start!!")
        self.RGB = QtWidgets.QLabel(self.centralwidget)
        self.RGB.setGeometry(QtCore.QRect(330, 500, 200, 50))
        self.RGB.setObjectName("RGB")
        self.VALUE = QtWidgets.QLabel(self.centralwidget)
        self.VALUE.setGeometry(QtCore.QRect(600, 500, 200, 50))
        self.VALUE.setObjectName("VALUE")
        self.close_btn = QtWidgets.QPushButton(self.centralwidget)
        self.close_btn.setGeometry(QtCore.QRect(1000, 500, 200, 100))
        font = QtGui.QFont()
        font.setFamily("微軟正黑體")
        font.setPointSize(18)
        font.setBold(False)
        font.setWeight(50)
        self.close_btn.setFont(font)
        self.close_btn.setObjectName("close_btn")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 887, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.img_output.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Output Image</p></body></html>"))
        self.img_output_2.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Output Image</p></body></html>"))
        self.start_btn.setText(_translate("MainWindow", "啟動"))
        self.RGB.setText(_translate("MainWindow", "Light"))
        self.VALUE.setText(_translate("MainWindow", "Value"))
        self.close_btn.setText(_translate("MainWindow", "關閉"))

