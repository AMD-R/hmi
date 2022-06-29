#!/usr/bin/env python3
import rospy
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from sympy import maximum

from service_node.srv import *
NAMESPACE = "/arm"
# SERVICE TOPIC
SERVICE_CMD = NAMESPACE + "/arm_cmd"
SERVICE_STOP = NAMESPACE + "/x_stop"

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState
from rosgraph_msgs.msg import Log

from hmi_modules.ctrl import Ui_ControlWindow
from hmi_modules.analoggaugewidget import QRoundProgressBar

class Ui_MainWindow(QMainWindow): ##object
    def openWindow(self):
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_ControlWindow()
        self.ui.setupUi(self.window)
        self.window.show()

    def setupUi(self, MainWindow):
        super().__init__()
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1269, 721)
        MainWindow.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(60, 30, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.myCall = QtWidgets.QPushButton(self.centralwidget)
        self.myCall.setGeometry(QtCore.QRect(50, 110, 291, 111))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myCall.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("parcel-removebg-preview.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        
        self.myCall.setIcon(icon)
        self.myCall.setIconSize(QtCore.QSize(40, 40))
        self.myCall.setObjectName("myCall")
        self.myCall.clicked.connect(self.call)

        self.myHome = QtWidgets.QPushButton(self.centralwidget)
        self.myHome.setGeometry(QtCore.QRect(50, 260, 291, 111))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myHome.setFont(font)

        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("home-removebg-preview.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.myHome.setIcon(icon1)
        self.myHome.setIconSize(QtCore.QSize(40, 40))
        self.myHome.setObjectName("myHome")
        self.myHome.show()
        self.myHome.clicked.connect(self.home)
        
        self.myManual = QtWidgets.QPushButton(self.centralwidget)
        self.myManual.setGeometry(QtCore.QRect(50, 400, 291, 241))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myManual.setFont(font)
        self.myManual.setObjectName("myManual")
        self.myManual.clicked.connect(self.openWindow)
        self.widget = QRoundProgressBar(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(500, 120, 321, 261))
        self.widget.setObjectName("widget")
    

        # Autoscrolling Logger
        self.widget_2 = QWidget(self.centralwidget) 
        self.widget_2.setGeometry(QtCore.QRect(370, 400, 561, 241))
        self.widget_2.setObjectName("widget_2")
        self.logText = QtWidgets.QLabel(self.widget_2)
        self.ScrollLabel = QScrollArea(self.widget_2)
        self.ScrollLabel.setGeometry(QtCore.QRect(0, 0, 561, 241)) ## this made the widget the desired sized.
        self.label_scroll = QtWidgets.QLabel(self.ScrollLabel)#??? ######
        self.testtext = "Logging..."
        self.label_scroll.text = self.testtext #delete this 
        self.label_scroll.setText(self.testtext)
        self.label_scroll.setGeometry(QtCore.QRect(0, 0, 561, 241))
        self.ScrollLabel.setWidgetResizable(True)
        self.lay = QVBoxLayout(self.widget_2)
        self.label_scroll.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.label_scroll.setWordWrap(True)
        self.lay.addWidget(self.label_scroll)
        self.ScrollLabel.setWidget(self.label_scroll)
        self.mybar = self.ScrollLabel.verticalScrollBar()
        self.mybar.setValue(self.mybar.maximum())
        self.widget_2.show()

        self.myStop = QtWidgets.QPushButton(self.centralwidget)
        self.myStop.setGeometry(QtCore.QRect(960, 120, 231, 171))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(166, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.HighlightedText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(166, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.HighlightedText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 25, 25))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(166, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.HighlightedText, brush)
        self.myStop.setPalette(palette)
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
        icon2.addPixmap(QtGui.QPixmap("stop-removebg-preview.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.myStop.setIcon(icon2)
        self.myStop.setIconSize(QtCore.QSize(1000, 1000))
        self.myStop.setObjectName("myStop")
        self.myStop.clicked.connect(self.stop)

        self.myTime = QtWidgets.QLabel(self.centralwidget)
        self.myTime.setGeometry(QtCore.QRect(960, 30, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.myTime.setFont(font)
        self.myTime.setObjectName("myTime")

        self.myMission = QtWidgets.QLabel(self.centralwidget)
        self.myMission.setGeometry(QtCore.QRect(380, 30, 551, 41))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.myMission.sizePolicy().hasHeightForWidth())
        self.myMission.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(20)
        self.myMission.setFont(font)
        self.myMission.setObjectName("myMission")
        #self.myMission.setText(self.label_scroll.text)
       
        self.myBatteryLevel = QtWidgets.QLabel(self.centralwidget)
        self.myBatteryLevel.setGeometry(QtCore.QRect(540, 90, 181, 16))
        self.myBatteryLevel.setAlignment(Qt.AlignCenter)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.myBatteryLevel.setFont(font)
        self.myBatteryLevel.setObjectName("myBatteryLevel")
        self.myReset = QtWidgets.QPushButton(self.centralwidget)
        self.myReset.setGeometry(QtCore.QRect(950, 400, 261, 241))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.myReset.setFont(font)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("../../../Downloads/reset-removebg-preview.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.myReset.setIcon(icon3)
        self.myReset.setIconSize(QtCore.QSize(40, 40))
        self.myReset.setObjectName("myReset")
        self.myReset.clicked.connect(self.show_pop)

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

    def day(self, temp):
        if temp == 0:
            return "Monday"
        elif temp == 1:
            return "Tuesday"
        elif temp == 2:
            return "Wednesday"
        elif temp == 3:
            return "Thursday"
        elif temp == 4:
            return "Friday"
        elif temp == 5:
            return "Saturday"
        elif temp == 6:
            return "Sunday"

    def updateTime(self, temp):
        self.temp = time.localtime()
        self.tempday = self.day(self.temp.tm_wday)
        if self.temp.tm_min < 10: 
            self.current_time = self.tempday + "  " + str(self.temp.tm_hour) + ":" + "0" + str(self.temp.tm_min)
        else:
            self.current_time = self.tempday + "  " + str(self.temp.tm_hour) + ":" + str(self.temp.tm_min)
        self.myTime.setText(str(self.current_time))
        self.myTime.update()
        self.ScrollLabel.verticalScrollBar().setValue(self.ScrollLabel.verticalScrollBar().maximum())

    def updateBattery(self, newbatt):
        self.widget.setValue(newbatt)
        self.update()

    def updateMission(self, newmission):
        self.myMission.setText("Current Mission: " + newmission)
        self.myMission.setAlignment(Qt.AlignCenter)
        
    def updateLog(self, newtext): #updates log widget text and autoscrolls
        # listq = ['a', 'b']
        # listq.append(newtext)
        # for i in range (1,5):
        #     print(i)
        #     self.label_scroll.text = self.label_scroll.text + "\n"
        self.label_scroll.text = self.label_scroll.text + "\n" + newtext #append new string
        self.label_scroll.setText(self.label_scroll.text)
        self.label_scroll.update() #update text
        self.label_scroll.setWordWrap(True)
        app.processEvents()
        self.ScrollLabel.verticalScrollBar().setValue(self.ScrollLabel.verticalScrollBar().maximum()) # + self.label_scroll.height())
    
    def clientVision(self, a):
        rate = rospy.Rate(1)
        try:        
            global visionResp

            rospy.wait_for_service('Vision')

            service = rospy.ServiceProxy(
                'Vision', Vision)
            visionResp = service(a)

            self.rate.sleep()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def clientArm(self, a, b, c, d, e, f):
        rate = rospy.Rate(1)
        global armResp
        try:
            rospy.wait_for_service(SERVICE_CMD)

            service = rospy.ServiceProxy(
                SERVICE_CMD, Arm)
            armResp = service(a, b, c, d, e, f)
            rate.sleep()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def clientNav(self, a, b, c):
        rate = rospy.Rate(1)
        global navResp
        try:
            rospy.wait_for_service('Nav')

            service = rospy.ServiceProxy(
                'Nav', Nav)
            navResp = service(a, b, c)
            self.updateLog("client nav running")
            rate.sleep()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def clientHMI(self, a, b, c):
        rate = rospy.Rate(1)
        global hmiResp
        try:
            rospy.wait_for_service('HMI')

            service = rospy.ServiceProxy(
                'HMI', HMI)
            hmiResp = service(a, b, c)
            rate.sleep()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def clientStop(self):
        rate = rospy.Rate(1)
        global stopResp
        try:
            rospy.wait_for_service(SERVICE_STOP)

            service = rospy.ServiceProxy(
                SERVICE_STOP, Stop)
            stopResp = service()
            rate.sleep()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def clientButton(self):
        rate = rospy.Rate(1)
        global buttonResp
        try:
            rospy.wait_for_service('buttonStatus')

            service = rospy.ServiceProxy(
                'buttonStatus', buttonStatus)
            buttonResp = service(True)
            rate.sleep()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def call(self):  
        self.newtext = "TRENT button pressed! Heading to Trent..."
        self.updateLog(self.newtext)
        self.clientVision(True)
        if visionResp.z == True:
            self.clientArm(0, visionResp.x, visionResp.y , 300, 120, False)
            time.sleep(26)
            self.clientArm(0, 0, 0 , 300, 120, True)

            self.clientButton()
            print(buttonResp)
            if buttonResp.isActive == True:
                self.clientStop()
                time.sleep(1)
                self.clientArm(0, -visionResp.x, -visionResp.y , 300, 120, False)
        
    def home(self):
        self.newtext = "HOME button pressed! Heading to Block D..."
        self.updateLog(self.newtext)

    def stop(self):
        self.newtext = "STOP button pressed!Cancelling all operations..."
        self.updateLog(self.newtext)
        # cancel nav goal, send cmd vel 0, 

    def reset(self):
        self.newtext = "Cancelling Desitnation..."
        self.updateLog(self.newtext)

    def show_pop(self):
        msg = QMessageBox()
        msg.setWindowTitle("Reset Window")
        msg.setText("Confirm Reset Destination?")
        msg.setIcon(QMessageBox.Question) #Warning, Question, Critical, Info
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        msg.setDefaultButton(QMessageBox.Ok) #default button with blue frame
        msg.setInformativeText("Click OK to proceed")
        #msg.setDetailedText("")
        msg.buttonClicked.connect(self.pop_button)
        x = msg.exec_() #show message box

    def pop_button(self, i): # i = widget that we clicked
        print(i.text())
        if i.text() == "&OK":
            print("selected OK")
            # cancel nav goal
        else: 
            print("selected Cancel")
            # go back to main window
        
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "ANDY's MAIL ROBOT"))
        self.myStop.setText(_translate("MainWindow", "STOP"))
        self.myCall.setText(_translate("MainWindow", " TRENT"))
        self.myHome.setText(_translate("MainWindow", " HOME"))
        self.myManual.setText(_translate("MainWindow", "MANUAL CONTROL"))
        self.myTime.setText(_translate("MainWindow", "Loading Time..."))
        self.myMission.setText(_translate("MainWindow", "Current Mission: "))
        self.myBatteryLevel.setText(_translate("MainWindow", "Battery Level"))
        self.myReset.setText(_translate("MainWindow", "RESET"))

def batteryTemp(data): # update battery on hmi
    ui.updateBattery(data.percentage)
    ui.updateTime(time.localtime())

def logTemp_rosout(data): # add debug msg on hmi from ros topics
    strdata = str(data.name)
    ui.updateLog(strdata)

def logTemp_order(data): # order node 
    if data == "MAIL":
        ui.updateLog("On my way to Trent!")
    elif data == "HOME":
        ui.updateLog("Back to Block D!")

if __name__ == "__main__":
    try:
        global app
        import sys
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        ui = Ui_MainWindow()
        ui.setupUi(MainWindow)
        MainWindow.show()
        
        rospy.init_node("Server")
        global visionResp, buttonResp, stopResp, hmiResp, navResp, armResp

        rospy.init_node('sub_batt', anonymous=True)
        rospy.Subscriber("battery", BatteryState, batteryTemp) # 
        rospy.Subscriber("rosout", Log, logTemp_rosout)
        rospy.Subscriber("order", String, logTemp_order)
        ui.updateLog("Log message updated..")
        ui.updateMission("OTW to Trent")

        sys.exit(app.exec())
    except rospy.ROSInterruptException:
        pass