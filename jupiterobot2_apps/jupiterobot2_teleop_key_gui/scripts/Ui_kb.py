#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/mustar/keyboard_gui/kb.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(834, 231)
        MainWindow.setStyleSheet("border-radius: 10px;")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.frame_2 = QtWidgets.QFrame(self.centralwidget)
        self.frame_2.setMinimumSize(QtCore.QSize(300, 200))
        self.frame_2.setStyleSheet("border-radius: 10px;  border: 2px groove gray;")
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.label = QtWidgets.QLabel(self.frame_2)
        self.label.setGeometry(QtCore.QRect(10, 180, 101, 31))
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.frame_2)
        self.label_3.setGeometry(QtCore.QRect(140, 180, 151, 31))
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.frame_2)
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setContentsMargins(0, -1, -1, -1)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy)
        self.pushButton.setMinimumSize(QtCore.QSize(50, 40))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.pushButton.setStyleSheet("QPushButton{\n"
"background-color: rgb(170, 170, 255);\n"
"border-radius: 10px;  \n"
"border: 2px groove gray;}\n"
"QPushButton:pressed {background-color: blue;}")
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 0, 1, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.frame)
        self.pushButton_5.setMinimumSize(QtCore.QSize(50, 40))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.pushButton_5.setStyleSheet("QPushButton{\n"
"background-color: rgb(170, 170, 255);\n"
"border-radius: 10px;  \n"
"border: 2px groove gray;}\n"
"QPushButton:pressed {background-color: blue;}")
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout.addWidget(self.pushButton_5, 1, 1, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.frame)
        self.pushButton_3.setMinimumSize(QtCore.QSize(50, 40))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.pushButton_3.setStyleSheet("QPushButton{\n"
"background-color: rgb(170, 170, 255);\n"
"border-radius: 10px;  \n"
"border: 2px groove gray;}\n"
"QPushButton:pressed {background-color: blue;}")
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 1, 0, 1, 1)
        self.pushButton_4 = QtWidgets.QPushButton(self.frame)
        self.pushButton_4.setMinimumSize(QtCore.QSize(50, 40))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.pushButton_4.setStyleSheet("QPushButton{\n"
"background-color: rgb(170, 170, 255);\n"
"border-radius: 10px;  \n"
"border: 2px groove gray;}\n"
"QPushButton:pressed {background-color: blue;}")
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout.addWidget(self.pushButton_4, 1, 2, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_2.sizePolicy().hasHeightForWidth())
        self.pushButton_2.setSizePolicy(sizePolicy)
        self.pushButton_2.setMinimumSize(QtCore.QSize(50, 40))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Condensed")
        font.setPointSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.pushButton_2.setStyleSheet("QPushButton{\n"
"background-color: rgb(170, 170, 255);\n"
"border-radius: 10px;  \n"
"border: 2px groove gray;}\n"
"QPushButton:pressed {background-color: blue;}")
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 2, 1, 1, 1)
        self.horizontalLayout.addLayout(self.gridLayout)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2.addWidget(self.frame)
        self.frame_3 = QtWidgets.QFrame(self.centralwidget)
        self.frame_3.setMinimumSize(QtCore.QSize(300, 200))
        self.frame_3.setStyleSheet("border-radius: 10px;  border: 2px groove gray;")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.label_2 = QtWidgets.QLabel(self.frame_3)
        self.label_2.setGeometry(QtCore.QRect(10, 180, 111, 31))
        self.label_2.setObjectName("label_2")
        self.label_4 = QtWidgets.QLabel(self.frame_3)
        self.label_4.setGeometry(QtCore.QRect(140, 180, 161, 31))
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.frame_3)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Teleop_Keyboard"))
        self.label.setText(_translate("MainWindow", "满速 0.26m/s"))
        self.label_3.setText(_translate("MainWindow", "当前线速度 00m/s"))
        self.pushButton.setText(_translate("MainWindow", "W"))
        self.pushButton_5.setText(_translate("MainWindow", "S"))
        self.pushButton_3.setText(_translate("MainWindow", "A"))
        self.pushButton_4.setText(_translate("MainWindow", "D"))
        self.pushButton_2.setText(_translate("MainWindow", "X"))
        self.label_2.setText(_translate("MainWindow", "满速1.82 rad/s"))
        self.label_4.setText(_translate("MainWindow", "当前角速度 00 rad/s"))
