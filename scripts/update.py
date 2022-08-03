#!/usr/bin/env python3
import rospy
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
import actionlib
from sympy import maximum
import iconfinal

from arm_controller.clients import *

import std_srvs.srv

from std_msgs.msg import String
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import BatteryState
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

from hmi_modules.ctrl import Ui_ControlWindow
from hmi_modules.analoggaugewidget import QRoundProgressBar


class Ui_MainWindow(QMainWindow):  # object

    def openWindow(self):
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_ControlWindow()
        self.ui.setupUi(self.window)
        self.window.show()
        self.close()

    def setupUi(self, MainWindow):
        super().__init__()

        # rospy.init_node('send_client_goal')
        self.client = actionlib.SimpleActionClient('/move_base',
                                                   MoveBaseAction)
        # self.client.wait_for_server()
        # MainWindow.showFullScreen()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_nav = rospy.Publisher('/move_base/cancel', GoalID,
                                       queue_size=10)
        self.cmdvel = Twist()

        self.pub_mission = rospy.Publisher("iot/mission", String,
                                           queue_size=10)
        self.curmission = String()

        self.clear_costmap_client = rospy.ServiceProxy(
            '/move_base/clear_costmaps', std_srvs.srv.Empty)

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 720)
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # Nottingham Logo
        self.myLogo = QtWidgets.QLabel(self.centralwidget)
        self.myLogo.setGeometry(QtCore.QRect(50, 10, 281, 121))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.myLogo.setFont(font)
        self.myLogo.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.myLogo.setLineWidth(0)
        self.myLogo.setText("")
        self.myLogo.setPixmap(QtGui.QPixmap(":/newicon/icons/nott.png"))
        self.myLogo.setObjectName("myLogo")

        # Button to go to trent
        self.myCall = QtWidgets.QPushButton(self.centralwidget)
        self.myCall.setGeometry(QtCore.QRect(50, 130, 281, 141))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myCall.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newicon/icons/trent.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.myCall.setIcon(icon)
        self.myCall.setIconSize(QtCore.QSize(40, 40))
        self.myCall.setObjectName("myCall")
        self.myCall.clicked.connect(self.call)

        # Button to go to D block
        self.myHome = QtWidgets.QPushButton(self.centralwidget)
        self.myHome.setGeometry(QtCore.QRect(50, 300, 281, 141))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myHome.setFont(font)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/newicon/icons/home.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.myHome.setIcon(icon1)
        self.myHome.setIconSize(QtCore.QSize(40, 40))
        self.myHome.setObjectName("myHome")
        self.myHome.show()
        self.myHome.clicked.connect(self.home)

        # Manual Controller for the AMD-R
        self.myManual = QtWidgets.QPushButton(self.centralwidget)
        self.myManual.setGeometry(QtCore.QRect(950, 315, 231, 171))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myManual.setFont(font)
        self.myManual.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/newicon/icons/control.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.myManual.setIcon(icon2)
        self.myManual.setIconSize(QtCore.QSize(150, 150))
        self.myManual.setObjectName("myManual")
        self.myManual.clicked.connect(self.openWindow)

        # Progress bar for battery percentage
        self.widget = QRoundProgressBar(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(535, 140, 261, 221))# 321, 261))
        # self.widget.setAlignment(Qt.AlignCenter)
        self.widget.setObjectName("widget")

        # Autoscrolling Logger
        self.widget_2 = QWidget(self.centralwidget) 
        self.widget_2.setGeometry(QtCore.QRect(360, 370, 561, 241))
        self.widget_2.setObjectName("widget_2")
        self.logText = QtWidgets.QLabel(self.widget_2)
        self.ScrollLabel = QScrollArea(self.widget_2)
        self.ScrollLabel.setGeometry(QtCore.QRect(0, 0, 561, 241)) ## this made the widget the desired sized.
        self.label_scroll = QtWidgets.QLabel(self.ScrollLabel)#??? ######
        self.testtext = "Logging..."
        self.label_scroll.text = self.testtext #delete this 
        self.label_scroll.setText(self.testtext)
        self.label_scroll.setGeometry(QtCore.QRect(0, 0, 561, 241))
        self.label_scroll_counter = QtWidgets.QLabel(self.ScrollLabel)
        self.ScrollLabel.setWidgetResizable(True)
        self.lay = QVBoxLayout(self.widget_2)
        self.label_scroll.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.label_scroll.setWordWrap(True)
        self.lay.addWidget(self.label_scroll)
        self.ScrollLabel.setWidget(self.label_scroll)
        self.mybar = self.ScrollLabel.verticalScrollBar()
        self.mybar.setValue(self.mybar.maximum())
        self.widget_2.show()

        # Emergency Stop button
        self.myStop = QtWidgets.QPushButton(self.centralwidget)
        self.myStop.setGeometry(QtCore.QRect(950, 130, 231, 171))
        self.myStop.setStyleSheet('QPushButton'
                                  '{background-color: #A3C1DA; color: red;}')
        # self.myStop.setStyleSheet("background-color: rgb(255, 0, 0);")
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(20)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.myStop.setFont(font)
        self.myStop.setStyleSheet("")
        self.myStop.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/newicon/icons/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.myStop.setIcon(icon2)
        self.myStop.setIconSize(QtCore.QSize(500, 500))
        # self.myStop.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.myStop.setObjectName("myStop")
        self.myStop.clicked.connect(self.stop)

        # Current Time Label
        self.myTime = QtWidgets.QLabel(self.centralwidget)
        self.myTime.setGeometry(QtCore.QRect(950, 50, 231, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.myTime.setFont(font)
        self.myTime.setAlignment(Qt.AlignCenter)
        self.myTime.setObjectName("myTime")

        # Current Mission Label
        self.myMission = QtWidgets.QLabel(self.centralwidget)
        self.myMission.setGeometry(QtCore.QRect(460, 20, 400, 91))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                           QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.myMission.sizePolicy()
                                     .hasHeightForWidth())
        self.myMission.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(20)
        self.myMission.setFont(font)
        self.myMission.setObjectName("myMission")
        # self.myMission.setText(self.label_scroll.text)

        # Battery Level Label
        self.myBatteryLevel = QtWidgets.QLabel(self.centralwidget)
        self.myBatteryLevel.setGeometry(QtCore.QRect(545, 110, 181, 16))
        self.myBatteryLevel.setAlignment(Qt.AlignCenter)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.myBatteryLevel.setFont(font)
        self.myBatteryLevel.setObjectName("myBatteryLevel")
        self.myReset = QtWidgets.QPushButton(self.centralwidget)
        self.myReset.setGeometry(QtCore.QRect(50, 470, 281, 141))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myReset.setFont(font)
        self.myReset.setObjectName("myReset")
        self.myReset.clicked.connect(self.demo_arm)

        # Quit Button
        self.myExit = QtWidgets.QPushButton(self.centralwidget)
        self.myExit.setGeometry(QtCore.QRect(1170, 0, 91, 61))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(":/newicon/icons/exit.png"),
                        QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.myExit.setIcon(icon3)
        self.myExit.setIconSize(QtCore.QSize(60, 60))
        self.myExit.clicked.connect(self.exitapp)
        self.myExit.setObjectName("myExit")

        self.myDesc = QtWidgets.QLabel(self.centralwidget)
        self.myDesc.setGeometry(QtCore.QRect(950, 500, 231, 111))
        self.myDesc.setFrameShape(QtWidgets.QFrame.Box)
        self.myDesc.setObjectName("myDesc")

        # Status Bar
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1269, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        self.haha = 0
        # myList[1] = "Logging Window"

    def updateTime(self):
        """Updates time label."""
        self.temp = time.localtime()
        self.current_time = time.strftime("%A %H:%M", self.temp)
        self.myTime.setText(str(self.current_time))
        self.myTime.update()
        self.ScrollLabel.verticalScrollBar().setValue(
            self.ScrollLabel.verticalScrollBar().maximum())

    def updateBattery(self, newbatt):
        """Updates the battery value label."""
        self.widget.setValue(newbatt)
        self.update()

    def updateMission(self, newmission):
        """Updates current mission label."""
        self.myMission.setText("Current Mission: " + newmission)
        self.myMission.setAlignment(Qt.AlignCenter)

    def updateLog(self, newtext):
        """Updates log widget text and autoscrolls."""
        self.haha += 1
        if self.haha >= 20:
            self.label_scroll.text = "Logging Window"
            self.haha = 0
            print(self.label_scroll.text)
        self.label_scroll.text = self.label_scroll.text + "\n" + newtext  # append new string
        self.label_scroll.setText(self.label_scroll.text)
        self.label_scroll.update()  # update text
        self.label_scroll.setWordWrap(True)
        app.processEvents()
        showing = str(self.label_scroll.text)
        self.ScrollLabel.verticalScrollBar().setValue(self.ScrollLabel.height())
        # self.ScrollLabel.verticalScrollBar().setValue(self.ScrollLabel.verticalScrollBar().maximum()) # + self.label_scroll.height())

    def exitapp(self):
        """Button to exit the UI."""
        sys.exit()
        
    def call(self):
        """Button to move AMD-R to trent."""
        self.newtext = "TRENT button pressed! Heading to Trent..."
        self.updateLog(self.newtext)
        self.updateMission("Heading to Lift")
        self.curmission = "1"
        self.pub_mission.publish(self.curmission)
        self.clear_costmap_client()
        time.sleep(2)
        # self.client.wait_for_server()
        # indication that server is online how
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = -24.203
        self.goal.target_pose.pose.position.y = 1.498
        self.goal.target_pose.pose.orientation.z = 1.000
        self.goal.target_pose.pose.orientation.w = 0.001
        self.client.send_goal(self.goal)
        while(self.client.get_state() == GoalStatus.PENDING) or (self.client.get_state() == GoalStatus.ACTIVE):
            app.processEvents()
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.updateLog("Approachnig destination")
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.pose.position.x = -28.314
            self.goal.target_pose.pose.position.y = 5.000
            self.goal.target_pose.pose.orientation.z = 1.000
            self.goal.target_pose.pose.orientation.w = 0.029
            self.client.send_goal(self.goal)
            while(self.client.get_state() == GoalStatus.PENDING) or (self.client.get_state() == GoalStatus.ACTIVE):
                app.processEvents()
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                print("Approaching destination")
                rospy.sleep(1)
                self.goal = MoveBaseGoal()
                self.goal.target_pose.header.frame_id = 'map' 
                self.goal.target_pose.pose.position.x = -29.571
                self.goal.target_pose.pose.position.y = 4.897
                self.goal.target_pose.pose.orientation.z = 1.000
                self.goal.target_pose.pose.orientation.w = 0.027
                self.client.send_goal(self.goal)
                while(self.client.get_state() == GoalStatus.PENDING) or (self.client.get_state() == GoalStatus.ACTIVE):
                    app.processEvents()
                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    self.updateLog("GOAL Reached.")
                    rospy.sleep(3)
                    self.demo_arm()
        else:
            self.updateLog("ERROR! GOAL not reached.")
            return

    def demo_arm(self):
        """Button to demo moving the robotic arm."""
        self.updateLog("ARM operation starting")
        self.updateMission("Hailing Lift")
        clientNav(True)
        clientVision(True)
        if visionResp.z == True:
            clientArm(0, visionResp.x, visionResp.y , 300, 200, False)
            timeY = visionResp.x*420
            timeZ = visionResp.y*160
            if timeY >= timeZ:
                time.sleep(timeY)
            elif timeZ >= timeY:
                time.sleep(timeZ)
            clientArm(0, 0, 0 , 300, 200, True)
            time.sleep(15)
            clientArm(0, 0, 0 , 300, 200, False)
            time.sleep(2)
            clientArm(0, -visionResp.x, -visionResp.y, 300, 200, False)
            if timeY >= timeZ:
                time.sleep(timeY)
            elif timeZ >= timeY:
                time.sleep(timeZ)

            count = 0
            while (count < 500):
                self.cmdvel.linear.x = -0.15
                self.cmdvel.linear.y = 0
                self.cmdvel.linear.z = 0
                self.cmdvel.angular.x = 0
                self.cmdvel.angular.y = 0
                self.cmdvel.angular.z = 0
                self.pub.publish(self.cmdvel)
                time.sleep(0.01)
                count += 1
            self.home()

    def demo(self):
        """Demo button for returning to HOME. (No longer used)."""
        self.updateLog("Arm operation here")
        rospy.sleep(5)
        self.home()

    def home(self):
        """Button to return to D Block."""
        # self.newtext = "HOME button pressed! Heading to Block D..."
        # self.updateLog(self.newtext)
        self.updateLog("Heading to HOME")
        self.updateMission("Heading to HOME")
        self.curmission = "0"
        self.pub_mission.publish(self.curmission)
        self.clear_costmap_client()
        time.sleep(2)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 5.762
        self.goal.target_pose.pose.position.y = -1.013
        self.goal.target_pose.pose.orientation.z = -0.060
        self.goal.target_pose.pose.orientation.w = 0.998
        self.client.send_goal(self.goal)
        while(self.client.get_state() == GoalStatus.PENDING) or (
                self.client.get_state() == GoalStatus.ACTIVE):
            app.processEvents()
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.updateLog("Approaching HOME")
            rospy.sleep(1)
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.pose.position.x = 8.242
            self.goal.target_pose.pose.position.y = -1.485
            self.goal.target_pose.pose.orientation.z = 0.696
            self.goal.target_pose.pose.orientation.w = 0.718
            self.client.send_goal(self.goal)
            while(self.client.get_state() == GoalStatus.PENDING) or (
                    self.client.get_state() == GoalStatus.ACTIVE):
                app.processEvents()
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.updateLog("Arrived at HOME.")
        else:
            self.updateLog("ERROR! HOME not reached.")
            return

    def stop(self):
        """Button to Emergency STOP."""
        self.newtext = "STOP button pressed! Cancelling all operations..."
        self.updateLog(self.newtext)
        self.cmdvel.linear.x = 0
        self.cmdvel.linear.y = 0
        self.cmdvel.linear.z = 0
        self.cmdvel.angular.x = 0
        self.cmdvel.angular.y = 0
        self.cmdvel.angular.z = 0
        self.pub.publish(self.cmdvel)
        self.pub_nav.publish()  # cancel
        # cancel nav goal, send cmd vel 0,

    def reset(self):
        """Buttton to reset destination (No Longer Used)."""
        self.newtext = "Cancelling Desitnation..."
        self.updateLog(self.newtext)

    def reset_pop(self):
        """Popup to confirm destination reset (No Longer Used)."""
        msg = QMessageBox()
        msg.setWindowTitle("Reset Window")
        msg.setText("Confirm Reset Destination?")

        # Warning, Question, Critical, Info
        msg.setIcon(QMessageBox.Question)
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)

        # default button with blue frame
        msg.setDefaultButton(QMessageBox.Ok)
        msg.setInformativeText("Choosing OK will cancel the current destination. Proceed?")
        msg.buttonClicked.connect(self.reset_pop_button)
        # show message box
        x = msg.exec_()

    def reset_pop_button(self, widget):
        """The Buttons for reset destination popup (No Longer Used)."""
        print(widget.text())
        if widget.text() == "&Yes":
            print("selected OK")
            self.pub_nav.publish()  # cancel
            # cancel nav goal
        else: 
            print("selected Cancel")
            # go back to main window
            
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        # self.label.setText(_translate("MainWindow", "ANDY's MAIL ROBOT"))
        # self.myStop.setText(_translate("MainWindow", "STOP"))
        self.myCall.setText(_translate("MainWindow", " GOAL"))
        self.myHome.setText(_translate("MainWindow", " HOME"))
        # self.myManual.setText(_translate("MainWindow", "MANUAL CONTROL"))
        self.myTime.setText(_translate("MainWindow", "Loading Time..."))
        self.myMission.setText(_translate("MainWindow",
                                          "Loading Current Mission..."))
        self.myBatteryLevel.setText(_translate("MainWindow", "Battery Level"))
        self.myReset.setText(_translate("MainWindow", "ARM"))
        # self.myExit.setText(_translate("MainWindow", "PushButton"))
        self.myDesc.setText(_translate("MainWindow",
                                       "EE Dept   AMD-R\n\n"
                                       "Jonathan Lee\n"
                                       "Kenji Eu\n"
                                       "Chia Yu Hang\n"
                                       "Neo Jie En"))


def batteryTemp(data: BatteryState, ui: Ui_MainWindow) -> None:
    """Callback function when data is recived from battery topic to update battery on hmi."""
    ui.updateBattery(data.percentage)
    ui.updateTime()


def logTemp_rosout(data: String):
    """Callback function to add debug msg on hmi from ros topics."""
    # strdata = str(data.data)
    # ui.updateLog("kO")
    ui.ScrollLabel.verticalScrollBar().setValue(
        ui.ScrollLabel.verticalScrollBar().maximum()
    )


def logTemp_order(data: Int16):
    """Callback function when an order is recived (Order node)."""
    ui.updateLog("here")
    tem = str(data.data)
    ui.updateLog(tem)
    if tem == "1":
        ui.updateLog("Heading to GOAL")
        ui.call()
    elif tem == "0":
        ui.updateLog("Heading to HOME")
        ui.home()

if __name__ == "__main__":
    try:
        global app
        import sys

        # Creating new node
        # rospy.init_node("sub_batt", anonymous=True)
        rospy.init_node('hmilog', anonymous=True)

        # Creating UI object
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        ui = Ui_MainWindow()
        ui.setupUi(MainWindow)
        MainWindow.showFullScreen()
        ui.updateMission("Idle")

        # Subscribing to topics
        # rospy.init_node('hmi_cmd_vel', anonymous=True)
        rospy.Subscriber("hmilog", String, logTemp_rosout)
        rospy.Subscriber("battery", BatteryState, batteryTemp, ui)
        rospy.Subscriber("iot/function", Int16, logTemp_order)

        sys.exit(app.exec())
    except rospy.ROSInterruptException:
        pass
