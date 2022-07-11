#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'control_panel_gui.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

import rospy
from erl_quadrotor_control.msg import panelcmd
from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def takeoffButton(self):
        print "Takeoff Button is clicked!"
        cmd.arm = True
        pub.publish(cmd)
    def landButton(self):
        print "Land Button is clicked!"
        cmd.arm = False
        pub.publish(cmd)
    def px4Button(self):
        print "PX4 Button is clicked!"
        cmd.algorithm = 1
        pub.publish(cmd)
    def geometricButton(self):
        print "Geometric Button is clicked!"
        cmd.algorithm = 2
        pub.publish(cmd)
    def hoverButton(self):
        print "Hover Button is clicked!"
        cmd.trajectory_mode = 1
        pub.publish(cmd)
    def circleButton(self):
        print "Circle Button is clicked!"
        cmd.trajectory_mode = 2
        pub.publish(cmd)
    def setpointButton(self):
        print "Setpoint Button is clicked!"
        cmd.trajectory_mode = 3
        cmd.x = float(self.textEdit.toPlainText())
        cmd.y = float(self.textEdit_2.toPlainText())
        cmd.z = float(self.textEdit_3.toPlainText())
        cmd.yaw = float(self.textEdit_4.toPlainText())
        pub.publish(cmd)
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(342, 462)
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(90, 10, 171, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.groupBox = QtGui.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(50, 50, 241, 81))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.pushButton = QtGui.QPushButton(self.groupBox)
        self.pushButton.setGeometry(QtCore.QRect(10, 40, 89, 25))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton_2 = QtGui.QPushButton(self.groupBox)
        self.pushButton_2.setGeometry(QtCore.QRect(140, 40, 89, 25))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.groupBox_2 = QtGui.QGroupBox(Form)
        self.groupBox_2.setGeometry(QtCore.QRect(50, 140, 241, 81))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.pushButton_3 = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_3.setGeometry(QtCore.QRect(10, 40, 89, 25))
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.pushButton_4 = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_4.setGeometry(QtCore.QRect(140, 40, 89, 25))
        self.pushButton_4.setObjectName(_fromUtf8("pushButton_4"))
        self.groupBox_3 = QtGui.QGroupBox(Form)
        self.groupBox_3.setGeometry(QtCore.QRect(50, 230, 241, 201))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.pushButton_5 = QtGui.QPushButton(self.groupBox_3)
        self.pushButton_5.setGeometry(QtCore.QRect(10, 40, 89, 25))
        self.pushButton_5.setObjectName(_fromUtf8("pushButton_5"))
        self.pushButton_6 = QtGui.QPushButton(self.groupBox_3)
        self.pushButton_6.setGeometry(QtCore.QRect(140, 40, 89, 25))
        self.pushButton_6.setObjectName(_fromUtf8("pushButton_6"))
        self.pushButton_7 = QtGui.QPushButton(self.groupBox_3)
        self.pushButton_7.setGeometry(QtCore.QRect(80, 170, 89, 25))
        self.pushButton_7.setObjectName(_fromUtf8("pushButton_7"))
        self.textEdit = QtGui.QTextEdit(self.groupBox_3)
        self.textEdit.setGeometry(QtCore.QRect(30, 110, 81, 21))
        self.textEdit.setObjectName(_fromUtf8("textEdit"))
        self.textEdit_3 = QtGui.QTextEdit(self.groupBox_3)
        self.textEdit_3.setGeometry(QtCore.QRect(150, 110, 81, 21))
        self.textEdit_3.setObjectName(_fromUtf8("textEdit_3"))
        self.textEdit_2 = QtGui.QTextEdit(self.groupBox_3)
        self.textEdit_2.setGeometry(QtCore.QRect(30, 140, 81, 21))
        self.textEdit_2.setObjectName(_fromUtf8("textEdit_2"))
        self.textEdit_4 = QtGui.QTextEdit(self.groupBox_3)
        self.textEdit_4.setGeometry(QtCore.QRect(150, 140, 81, 21))
        self.textEdit_4.setObjectName(_fromUtf8("textEdit_4"))
        self.label_2 = QtGui.QLabel(self.groupBox_3)
        self.label_2.setGeometry(QtCore.QRect(10, 110, 16, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.groupBox_3)
        self.label_3.setGeometry(QtCore.QRect(10, 140, 16, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.groupBox_3)
        self.label_4.setGeometry(QtCore.QRect(130, 110, 16, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.groupBox_3)
        self.label_5.setGeometry(QtCore.QRect(130, 140, 21, 17))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.label_6 = QtGui.QLabel(self.groupBox_3)
        self.label_6.setGeometry(QtCore.QRect(70, 80, 121, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL(_fromUtf8("clicked()")), self.takeoffButton)
        QtCore.QObject.connect(self.pushButton_2, QtCore.SIGNAL(_fromUtf8("clicked()")), self.landButton)
        QtCore.QObject.connect(self.pushButton_3, QtCore.SIGNAL(_fromUtf8("clicked()")), self.px4Button)
        QtCore.QObject.connect(self.pushButton_4, QtCore.SIGNAL(_fromUtf8("clicked()")), self.geometricButton)
        QtCore.QObject.connect(self.pushButton_5, QtCore.SIGNAL(_fromUtf8("clicked()")), self.hoverButton)
        QtCore.QObject.connect(self.pushButton_6, QtCore.SIGNAL(_fromUtf8("clicked()")), self.circleButton)
        QtCore.QObject.connect(self.pushButton_7, QtCore.SIGNAL(_fromUtf8("clicked()")), self.setpointButton)

        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.label.setText(_translate("Form", "ERL Quadrotor Control", None))
        self.groupBox.setTitle(_translate("Form", "                Quadrotor Arming   ", None))
        self.pushButton.setText(_translate("Form", "Takeoff", None))
        self.pushButton_2.setText(_translate("Form", "Land", None))
        self.groupBox_2.setTitle(_translate("Form", "                Control Algorithm  ", None))
        self.pushButton_3.setText(_translate("Form", "PX4", None))
        self.pushButton_4.setText(_translate("Form", "Geometric", None))
        self.groupBox_3.setTitle(_translate("Form", "                        Trajectory     ", None))
        self.pushButton_5.setText(_translate("Form", "Hover", None))
        self.pushButton_6.setText(_translate("Form", "Circle", None))
        self.pushButton_7.setText(_translate("Form", "Setpoint", None))
        self.label_2.setText(_translate("Form", "x:", None))
        self.label_3.setText(_translate("Form", "y:", None))
        self.label_4.setText(_translate("Form", "z:", None))
        self.label_5.setText(_translate("Form", "Ψ:", None))
        self.label_6.setText(_translate("Form", " Enter a setpoint", None))

def pub_cb(self):
    pub.publish(cmd)
if __name__ == "__main__":
    import sys
    cmd = panelcmd()
    # Default values 
    cmd.arm = False
    cmd.algorithm = 0
    cmd.trajectory_mode = 0
    cmd.x = 0
    cmd.y = 0
    cmd.z = 3
    cmd.yaw = 0 
    pub = rospy.Publisher('erl_quadrotor_control/quadrotor_gui_cmd', panelcmd, queue_size=10)
    rospy.init_node('quadrotor_gui_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    rospy.Timer(rospy.Duration(0.1), pub_cb)
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

