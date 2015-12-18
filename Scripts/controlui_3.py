#!/usr/bin/env python

import rospy
import os

import Queue
import threading
import thread
import signal
import sys
import numpy as np
import dynamic_reconfigure.client

from PyQt4 import Qt
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import random

class controlui(QMainWindow):
    
    update_freq = 40
    #data = None
    
    data = {'goal': 0,'depth_kp' : 0, 'heading_kp' : 0, 'depth_motor_kp': 0, 'heading_motor_kp': 0, 'depth_startup_time': 0, 'heading_startup_time': 0}
    
    def __init__(self, parent=None):
        super(controlui, self).__init__(parent)
        
        #self.main_tab = QTabWidget()
        self.main_frame = QWidget()
        #self.main_tab.addTab(self.main_frame, "Tuning")
        self.main_layout = QHBoxLayout()
        
        #============================Graph widget creation=====================================
        GraphBox =  QGroupBox("Graph")
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.refresh = QPushButton('Refresh',self)
        self.refresh.clicked.connect(self.plot)
        
        GraphBox_layout = QVBoxLayout()
        GraphBox_layout.addWidget(self.refresh)
        GraphBox_layout.addWidget(self.canvas)
        
        GraphBox.setLayout(GraphBox_layout)
        #============================Telemetry widget creation=====================================
        #Info
        TelemetryBox = QGroupBox("Telemetry")
        self.telemetryPanel1 = QTextBrowser()
        self.telemetryPanel1.setStyleSheet("QTextBrowser { background-color : darkred; color :white; }")
        self.telemetryPanel2 = QTextBrowser()
        self.telemetryPanel2.setStyleSheet("QTextBrowser { background-color : darkred; color :white; }")
        self.telemetryPanel3 = QTextBrowser()
        self.telemetryPanel3.setStyleSheet("QTextBrowser { background-color : darkred; color :white; }")
        telemetry_layout = QHBoxLayout()
        telemetry_layout.addWidget(self.telemetryPanel1)
        telemetry_layout.addWidget(self.telemetryPanel2)
        telemetry_layout.addWidget(self.telemetryPanel3)
        TelemetryBox.setLayout(telemetry_layout)
        #============================Control Parameters=============================================
        ParamBox = QGroupBox("Control Parameters")
        D_Kp , self.depth_kp_box, depth_Kp_layout = self.make_data_box("Depth Kp: ")
        H_Kp , self.heading_kp_box, heading_Kp_layout = self.make_data_box("Yaw Kp:  ")
        DM_Kp , self.depth_motor_kp_box, depth_motor_Kp_layout = self.make_data_box("DepthM Kp:")
        HM_Kp , self.heading_motor_kp_box, heading_motor_Kp_layout = self.make_data_box("YawM Kp:")
        HT , self.HT_box, HT_layout = self.make_data_box("Heading Time: ")
        DT , self.DT_box, DT_layout = self.make_data_box("Depth Time:")
        Goal , self.Goal_box, R_layout = self.make_data_box("Goal:   ")
        Yaw_chk, self.Yaw_chkbox, Yaw_chk_layout = self.make_data_chkbox("Yaw     ")
        Depth_chk, self.Depth_chkbox, Depth_chk_layout = self.make_data_chkbox("Depth")
        
        paramb = QPushButton('Tune!', self)
        paramb.clicked.connect(self.update)
        
        p1_layout = QHBoxLayout()
        p1_layout.addLayout(depth_Kp_layout)
        p1_layout.addLayout(HT_layout)
        p2_layout = QHBoxLayout()
        p2_layout.addLayout(heading_Kp_layout)
        p2_layout.addLayout(DT_layout)
        p3_layout = QVBoxLayout()
        p3_layout.addLayout(R_layout)
        p3_layout.addLayout(Yaw_chk_layout)
        p3_layout.addLayout(Depth_chk_layout)
        p4_layout = QHBoxLayout()
        p4_layout.addLayout(depth_motor_Kp_layout)
        p4_layout.addWidget(paramb)
        p5_layout = QHBoxLayout()
        p5_layout.addLayout(heading_motor_Kp_layout)
        param_layout = QVBoxLayout()
        param_layout.addLayout(p1_layout)
        param_layout.addLayout(p2_layout)
        param_layout.addLayout(p4_layout)
        param_layout.addLayout(p5_layout)
        
        H_layout = QHBoxLayout()
        H_layout.addLayout(p3_layout)
        H_layout.addLayout(param_layout)
        ParamBox.setLayout(H_layout)
        #============================Main layout====================================================
        #Set all the layouts into the main layout. Postioning is determined by order of which widget is added first 
        Vlayout = QVBoxLayout()
        Vlayout.addWidget(ParamBox)
        Vlayout.addWidget(TelemetryBox)
        #Vlayout.addWidget(GoalBox)
        #Vlayout.addWidget(ParamBox)
        
        main_layout = QHBoxLayout()
        main_layout.addWidget(GraphBox)
        main_layout.addLayout(Vlayout)
        self.main_frame.setLayout(main_layout)
        #========================= End of main_layout==============================================
        
        #===================================Background colour====================================== 
        #self.setStyleSheet('QMainWindow{background-color: darkred;border: white;}')
        #==========================================================================================
        
        
        #========================= Main window setup ==============================================
        self.setCentralWidget(self.main_frame)
        self.setGeometry(300, 300, 1090, 760)
        self.setWindowTitle('Tuning')
        #self.setWindowIcon(QIcon(os.getcwd() + "/goal.png"))
        #self.initSub()
        #self.initTimer(self.rate) 
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.on_timer)
        self.timer.start(1000.0 / self.update_freq)
        #=============================End of Gui design============================================
        
    def on_timer(self):
        self.telemetryPanel1.setText("<b>Depth Setpt: "+ str(0) +
                                     "<br>Depth feedback: "+ str(0) +
                                     "<br>Depth Error: "+ str(0) +
                                    "<p>Yaw Setpt: " + str(0) + 
                                    "<br>Yaw feedback: "+ str(0) +
                                    "<br>Yaw Error: "+ str(0) + "</b>")
        self.telemetryPanel2.setText("<b>p: "+ str(0) +
                                    "<br>i: "+ str(0) +
                                    "<br>d: "+ str(0) + "</b>")
        self.telemetryPanel3.setText("<b> HL GOAL: " + str(0) +
                                    "<br> HR GOAL: " + str(0) +
                                    "<br> VL GOAL: " + str(0) +
                                    "<br> VR GOAL: " + str(0) +
                                    "<p>  HL: " + str(0) +
                                    "<br> HR: " + str(0) +
                                    "<br> VL: " + str(0) +
                                    "<br> VR: " + str(0) + "</b>")
        
    def update(self):
        #Depth Goal
        if self.Goal_box.text() == "":
            self.Goal_box.setText(str(0))
            set_Goal = 0
        else:
            set_Goal = float(self.Goal_box.text())
            self.Goal_box.setText(str(set_Goal))
            self.data['goal'] = set_Goal
            
        '''#Depth Heading
        if self.goal_heading_box.text() == "":
            self.goal_heading_box.setText(str(0))
            set_goal_heading = 0
        else:
            set_goal_heading  = float(self.goal_heading_box.text())
            self.goal_heading_box.setText(str(set_goal_heading))
            self.data['goal_heading'] = set_goal_heading'''
            
        #depth_startup_time
        if self.DT_box.text() == "":
            self.DT_box.setText(str(0))
            set_depth_startup_time = 0
        else:
            set_depth_startup_time = float(self.DT_box.text())
            self.DT_box.setText(str(set_depth_startup_time))
            self.data['depth_startup_time'] = set_depth_startup_time
            
        #heading_startup_time
        if self.HT_box.text() == "":
            self.HT_box.setText(str(0))
            set_heading_startup_time = 0
        else:
            set_heading_startup_time = float(self.HT_box.text())
            self.HT_box.setText(str(set_heading_startup_time))
            self.data['heading_startup_time'] = set_heading_startup_time
        
        #depth_kp
        if self.depth_kp_box.text() == "":
            self.depth_kp_box.setText(str(0))
            set_depth_kp = 0
        else:
            set_depth_kp = float(self.depth_kp_box.text())
            self.depth_kp_box.setText(str(set_depth_kp))
            self.data['depth_kp'] = set_depth_kp
            
        #heading_kp
        if self.heading_kp_box.text() == "":
            self.heading_kp_box.setText(str(0))
            set_heading_kp = 0
        else:
            set_heading_kp = float(self.heading_kp_box.text())
            self.heading_kp_box.setText(str(set_heading_kp))
            self.data['heading_kp'] = set_heading_kp
            
        #depth_motor_kp
        if self.depth_motor_kp_box.text() == "":
            self.depth_motor_kp_box.setText(str(0))
            set_depth_motor_kp = 0
        else:
            set_depth_motor_kp = float(self.depth_motor_kp_box.text())
            self.depth_motor_kp_box.setText(str(set_depth_motor_kp))
            self.data['depth_motor_kp'] = set_depth_motor_kp

        #heading_motor_kp
        if self.heading_motor_kp_box.text() == "":
            self.heading_motor_kp_box.setText(str(0))
            set_heading_motor_kp = 0
        else:
            set_heading_motor_kp= float(self.heading_motor_kp_box.text())
            self.heading_motor_kp_box.setText(str(set_heading_motor_kp))
            self.data['heading_motor_kp'] = set_heading_motor_kp
            
        self.updateConfig(set_Goal, set_depth_kp, set_heading_kp, set_depth_motor_kp, set_heading_motor_kp, set_depth_startup_time, set_heading_startup_time)
            
        
    def make_data_box(self, name):
        label = QLabel(name)
        label.setStyleSheet("QLabel { background-color : darkred; color :white; }")
        qle = QLineEdit()
        qle.setStyleSheet("QLineEdit { background-color : black; color :yellow; }")
        layout = QHBoxLayout()
        layout.addWidget(label)
        layout.addWidget(qle)
        layout.addStretch(1)
        qle.setFrame(False)

        return (label, qle, layout)
    
    def make_data_chkbox(self, name):
        label = QLabel(name)
        qle = QCheckBox()
        layout = QHBoxLayout()
        #qle.setEnabled(False)
        layout.addWidget(label)
        layout.addWidget(qle)
        layout.addStretch(1)

        return (label, qle, layout)
    
        
    def plot(self):
        x,y = np.loadtxt('newdata.txt', dtype=int,delimiter=',',unpack=True,usecols=(0,1))
        ax = self.figure.add_subplot(111)
        ax.plot(x,y)
        self.canvas.draw()
        
    def tune(self):
        exit(0)
        
    def goal(self):
        exit(0)
        
    def updateConfig(self,goal, depth_kp, heading_kp, depth_motor_kp, heading_motor_kp, depth_startup_time, heading_startup_time):
        #Update Config file parameters
        client = dynamic_reconfigure.client.Client('controller')
        pidparams = { 'goal' : goal , 'depth_kp' : depth_kp , 'heading_kp' : heading_kp, 'depth_motor_kp' : depth_motor_kp, 'heading_motor_kp' : heading_motor_kp, 'depth_startup_time' : depth_startup_time, 'heading_startup_time' : heading_startup_time}
        config = client.update_configuration(pidparams)
                
          
if __name__ == "__main__":
    #rospy.init_node('controlui', anonymous=True)
    app = QApplication(sys.argv)
    form = controlui()
    #signal.signal(signal.SIGINT, form.signal_handler)
    form.show()
    app.exec_()