#!/usr/bin/env python

import rospy
import os

import Queue
import threading
import thread
import signal
import sys
import dynamic_reconfigure.client

from std_msgs.msg import String
from std_msgs.msg._Float32 import Float32
from std_msgs.msg._Int16 import Int16

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 

import math
import random

from PyQt4 import Qt
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt


compass_data = 0.0

from vision_dynreconf import Vision
from controlui_3 import controlui

class AUV_gui(QMainWindow):
    #====Variables Declaration
    compass = None
    heading_provider = None
    yaw = 0
    update_freq = 40
    isFront = 0
    cvRGBImg_front = None
    cvRGBImg_rfront = None
    cvRGBImg_bot = None
    cvRGBImg_f_bot = None
    cvRGBImg_f_frt = None
    q_image_bot = None
    q_sonar = None
    q_image_front = None
    q_image_rfront = None
    vision_filter_frame = None
    filter_image = None
    frontcam_sub = None
    botcam_sub = None
    filter_sub = None
    frontfilter_sub = None
    botfilter_sub = None
    default_auv_ip = "192.168.2.1"
    default_host_ip = "192.168.2.2"
    #========callback storage
    q_depth = Queue.Queue()
    q_heading = Queue.Queue()
    q_Task_id = Queue.Queue()
    q_vrm = Queue.Queue()
    q_vlm = Queue.Queue()
    q_hrm = Queue.Queue()
    q_hlm = Queue.Queue()
    #=========Qtextbrowser variable declaration
    data = {'yaw': 0,'pitch' : 0,'roll' : 0, 'depth' : 0, 'heading_setpoint': 0, 'depth_setpoint': 0, 'VL': 0, 'VR': 0, 'HR': 0, 'HL': 0, 'SetupTime': 0, 'Duration': 0, 'ID': None}
    def __init__(self, parent=None):
        super(AUV_gui, self).__init__(parent)
        
        #===========================Tab widget creation ==========================================
        self.main_tab = QTabWidget()
        self.main_frame = QWidget()
        self.vision_frame = Vision()
        self.control_frame = controlui()
        self.main_tab.addTab(self.main_frame, "Telemetry")
        self.main_tab.addTab(self.vision_frame, "Vision")
        self.main_tab.addTab(self.control_frame, "Control")
        #============================End of Tab widget============================================
        
        #============================Settings creation============================================
        setupBox =  QGroupBox("Setup")
        #setupBox.setStyleSheet('QGroupBox {color: white background-color: black; border: white;}')
        #Need to remove the rest add only heading,depth, setup time, time limit
        
        depth_val , self.depth_val_box, depth_val_layout = self.make_data_box("Depth:")
        heading_val, self.heading_val_box,heading_val_layout = self.make_data_box("Heading:")
        SetupTime_val , self.SetupTime_box, SetupTime_layout = self.make_data_box("SetupTime:")
        duration_val , self.duration_box, duration_layout = self.make_data_box("Duration:")
        # Horizontal Thrusters
        HL_val, self.HL_val_box,HL_val_layout = self.make_data_box("HorL:   ")
        HR_val, self.HR_val_box,HR_val_layout = self.make_data_box("HorR:      ")
        # Vertical Thrusters
        VL_val, self.VL_val_box,VL_val_layout = self.make_data_box("VertL:           ")
        VR_val, self.VR_val_box,VR_val_layout = self.make_data_box("VertR:      ")
        
        updateb = QPushButton('&configure', self)
        updateb.clicked.connect(self.update)
        
        essentialsetup_layout = QHBoxLayout()
        essentialsetup_layout.addLayout(depth_val_layout)
        essentialsetup_layout.addLayout(heading_val_layout)
        essentialsetup_layout.addLayout(SetupTime_layout)
        essentialsetup_layout.addLayout(duration_layout)
        thrustersetup_layout = QHBoxLayout()
        thrustersetup_layout.addLayout(HL_val_layout)
        thrustersetup_layout.addLayout(HR_val_layout)
        thrustersetup_layout.addLayout(VL_val_layout)
        thrustersetup_layout.addLayout(VR_val_layout)
        mainsetup_layout = QVBoxLayout()
        mainsetup_layout.addLayout(essentialsetup_layout)
        mainsetup_layout.addLayout(thrustersetup_layout)
        mainsetup_layout.addWidget(updateb)

        setupBox.setLayout(mainsetup_layout)
        
        #setupBox.setStyleSheet('QGroupBox {background-color: black; border: white;}')
        #============================End of widget creation=======================================
        #============================ROS MASTER URI widget creation=====================================
        rosuriBox =  QGroupBox("ROS URI")
        AUV_IP , self.AUV_IP_box, AUV_IP_layout = self.make_data_box("AUV IP:")
        Host_IP , self.Host_IP_box, Host_IP_layout = self.make_data_box("Host IP:")
        
        
        ros_urib = QPushButton('&ROS URI', self)
        ros_urib.clicked.connect(self.rosuri)
        
        rosuri_layout = QVBoxLayout()
        rosuri_layout.addLayout(AUV_IP_layout)
        rosuri_layout.addLayout(Host_IP_layout)
        rosuri_layout.addWidget(ros_urib)
        
        rosuriBox.setLayout(rosuri_layout)

        #============================Attitude widget creation=====================================
    
        #attitude information
        attitudeBox = QGroupBox("Attitude")
        self.attitudePanel1 = QTextBrowser()
        self.attitudePanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        self.attitudePanel2 = QTextBrowser()
        self.attitudePanel2.setStyleSheet("QTextBrowser { background-color : black; color :white; }")

        attitude_layout = QVBoxLayout()
        attitude_layout.addWidget(self.attitudePanel1)
        attitude_layout.addWidget(self.attitudePanel2)
        attitudeBox.setLayout(attitude_layout)
        
        #Setpoint information
        setpointBox = QGroupBox("Setpoint")
        self.setpointPanel1 = QTextBrowser()
        self.setpointPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        setpoint_layout = QVBoxLayout()
        setpoint_layout.addWidget(self.setpointPanel1)
        setpointBox.setLayout(setpoint_layout)
        
        #Time information
        StatusBox = QGroupBox("Status")
        self.StatusPanel1 = QTextBrowser()
        self.StatusPanel1.setStyleSheet("QTextBrowser { background-color : black; color :white; }")
        Status_layout = QVBoxLayout()
        Status_layout.addWidget(self.StatusPanel1)
        StatusBox.setLayout(Status_layout)
        
        Info_layout = QVBoxLayout()
        Info_layout.addWidget(setpointBox)
        Info_layout.addWidget(StatusBox)
        #============================End of widget creation=======================================
        
        #============================Compass widget===============================================
        compass_box = QGroupBox("AUV Heading")
        self.compass = Qwt.QwtCompass()
        self.compass.setGeometry(0,0,190,190)
        self.compass.setLineWidth(4)
        self.compass.setMode(Qwt.QwtCompass.RotateNeedle)
        rose = Qwt.QwtSimpleCompassRose(16, 2)
        rose.setWidth(0.15)
        self.compass.setRose(rose)
        self.compass.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.ThinStyle))
        self.compass.setValue(270.0)
        self.compass.setScale(36, 5, 0)

        self.heading_provider = Qwt.QwtCompass()
        self.heading_provider.setLineWidth(4)
        self.heading_provider.setMode(Qwt.QwtCompass.RotateNeedle)
        rose = Qwt.QwtSimpleCompassRose(16, 2)
        rose.setWidth(0.15)
        self.heading_provider.setRose(rose)
        self.heading_provider.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.ThinStyle))

        compass_l = QLabel("Current")
        compass_l.setStyleSheet("QLabel { background-color : darkred; color :white; }")
        heading_l = QLabel("User Goal")
        heading_l.setStyleSheet("QLabel { background-color : darkred; color :white; }")
        compass_l.setAlignment(Qt.AlignHCenter)
        heading_l.setAlignment(Qt.AlignHCenter)

        compass_layout = QHBoxLayout()
        current_layout = QVBoxLayout()
        current_layout.addWidget(self.compass)
        current_layout.addWidget(compass_l)
        user_layout = QVBoxLayout()
        user_layout.addWidget(self.heading_provider)
        user_layout.addWidget(heading_l)
        compass_layout.addLayout(current_layout)
        compass_layout.addLayout(user_layout)
        
        compass_box.setLayout(compass_layout)
        #============================End of widget creation=======================================
        #============================Depth gauge widget===============================================
        #Depth Scale
        self.depth_thermo = Qwt.QwtThermo()
        self.depth_thermo.setPipeWidth(6)
        self.depth_thermo.setRange(0, 5)
        self.depth_thermo.setFillColor(Qt.green)
        self.depth_thermo.setAlarmColor(Qt.red)
        
        #============================Start/Stop button widget creation============================
    
        basic_box = QGroupBox("Basic Controls")
        engageb = QPushButton('&Start', self)
        disarmb = QPushButton('&Disarm', self)
       
        #self.pubEn = rospy.Publisher("Engage_btn", String)
        #self.pubDis = rospy.Publisher("Disarm_btn", String)

        engageb.clicked.connect(self.engage)
        disarmb.clicked.connect(self.disarm)
         
        basic_layout = QVBoxLayout()
        basic_layout.addWidget(engageb)
        basic_layout.addWidget(disarmb)
        
        basic_box.setLayout(basic_layout)
        
        #============================End of widget creation=======================================
        #============================Camera Selection=============================================
        ### Camera Imagery Declaration
        cam_index_layout = QVBoxLayout()
        self.cam_index_l = QLabel("<b>Camera Selection</b>")
        self.cam_index_cb = QComboBox()
        self.cam_index_cb.addItem("Front Camera")
        self.cam_index_cb.addItem("Bottom Camera")
        self.cam_index_cb.activated[int].connect(self.onActivated)
        #==========================================================================================
        #============================Camera widget creation========================================
        # Vision image layout
        camera_layout = QGroupBox("Video Feed") 
        front_video_l = QLabel("<b>Front Camera</b>")
        bot_video_l = QLabel("<b>Bottom Camera</b>")
        self.video_top = QLabel()
        self.video_bot = QLabel()

        video_layout = QVBoxLayout()
        video_layout.addWidget(self.cam_index_l)
        video_layout.addWidget(front_video_l)
        video_layout.addWidget(self.video_top)
        video_layout.addWidget(bot_video_l)
        video_layout.addWidget(self.video_bot)
        
        vision_layout = QVBoxLayout()
        vision_layout.addWidget(self.cam_index_cb)
        vision_layout.addLayout(video_layout)
        
        camera_layout.setLayout(vision_layout)
        #=========================================================================================
        
        #============================Main layout==================================================
        #Set all the layouts into the main layout. Postioning is determined by order of which widget is added first 
    
        H2layout = QHBoxLayout()
        H2layout.addWidget(attitudeBox)
        H2layout.addLayout(Info_layout)
        V1layout = QVBoxLayout()
        V1layout.addWidget(compass_box)
        V1layout.addLayout(H2layout)
        
        H3layout = QHBoxLayout()
        H3layout.addWidget(self.depth_thermo)
        H3layout.addWidget(camera_layout)
        
        HSlayout = QHBoxLayout()
        HSlayout.addWidget(basic_box)
        HSlayout.addWidget(setupBox)
        HSlayout.addWidget(rosuriBox)
        
        HMlayout = QHBoxLayout()
        HMlayout.addLayout(V1layout)
        HMlayout.addLayout(H3layout)
        
        main_layout = QVBoxLayout()
        main_layout.addLayout(HMlayout)
        main_layout.addLayout(HSlayout)
        self.main_frame.setLayout(main_layout)
        #========================= End of main_layout==============================================
        
        #===================================Background colour======================================
        #self.setStyleSheet('QMainWindow{background-color: darkred; border: white;}')
        #==========================================================================================
        
        
        #========================= Main window setup ==============================================
        self.setCentralWidget(self.main_tab)
        self.setGeometry(300, 300, 1090, 760)
        self.setWindowTitle('Mission Control')
        self.setWindowTitle('SPAUV Mission Control')
        #self.setWindowIcon(QIcon(os.getcwd() + '/icons/field.png'))
        self.heading_provider.valueChanged.connect(self.valueChanged)
        self.initImage()
        self.initSub()
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL('timeout()'), self.on_timer)
        #self.step = 1000.0 / self.update_freq
        self.timer.start(1000.0 / self.update_freq)
        #self.timer.start(self.step)
        #self.pbar.setValue(self.step)
        #=============================End of Gui design============================================
        
    #=============================QT & ROS event handler==================================================  
    #Major to do!!! Gui is done for now. left wit these ros signals :C
    def on_timer(self):
        heading = None
        depth = None
        Task_id = None
        vrm = None
        vlm = None
        hrm = None
        hlm = None
        
        image_front = None
        image_rfront = None
        f_image_bot = None
        f_image_front = None

        
        #Collection of data from ros server =================================
        '''Catch if queue is Empty exceptions'''
        try:
            heading = self.q_heading.get(False,0)
            self.q_heading = Queue.Queue()
        except Exception,e:
            pass
        try:
            depth = self.q_depth.get(False,0)
            self.q_depth = Queue.Queue()
        except Exception,e: 
            pass
        try:
            image_bot = self.q_image_bot
        except Exception,e:
            pass
        try:
            image_front = self.q_image_front
        except Exception,e:
            pass
        try:
            Task_id = self.q_Task_id.get(False,0)
            self.q_Task_id = Queue.Queue()
        except Exception,e: 
            pass
        try:
            vrm = self.q_vrm.get(False,0)
            self.q_vrm = Queue.Queue()
        except Exception,e:
            pass
        try:
            vlm = self.q_vlm.get(False,0)
            self.q_vlm = Queue.Queue()
        except Exception,e:
            pass
        try:
            hrm = self.q_hrm.get(False,0)
            self.q_hrm = Queue.Queue()
        except Exception,e:
            pass
        try:
            hlm = self.q_hlm.get(False,0)
            self.q_hlm = Queue.Queue()
        except Exception,e:
            pass
        
        '''If data in queue is available store it into data'''
        if heading != None:
            self.data['yaw'] = heading.data
        if depth != None:
            self.data['depth'] = depth.data
        if self.q_image_front != None:
            self.update_video_front(image_front)
        if image_bot != None:
            self.update_video_bot(image_bot)
        if vrm != None:
            self.data['VR'] = vrm.data
        if vlm != None:
            self.data['VL'] = vlm.data
        if hrm != None:
            self.data['HR'] = hrm.data
        if hlm != None:
            self.data['HL'] = hlm.data
        if Task_id != None:
            self.data['ID'] = Task_id.data
        
            
        #====================== End of collection ================================
        
        self.depth_thermo.setValue(round(self.data['depth'],2))
        self.compass.setValue(int(self.data['yaw']))
        
        self.attitudePanel2.setText("<b>VR: "+ str(round(self.data['VR'],2)) +
                                    "<br> VL: " + str(round(self.data['VL'],2)) +
                                    "<br> HR: " + str(round(self.data['HR'],2)) + 
                                    "<br> HL: " + str(round(self.data['HL'],2)) +"</b>")
        
        self.attitudePanel1.setText("<b>YAW: " + str(round(self.data['yaw'],2)) +
                                    "<br>DEP: "+ str(round(self.data['depth'],2)) + "</b>")
        
        self.setpointPanel1.setText("<b>HDG: " + str(round(self.data['heading_setpoint'],2)) +
                                    "<br>DEP: "+ str(round(self.data ['depth_setpoint'],2)) + "</b>")
        
        self.StatusPanel1.setText("<b>Task ID: "+ str(self.data['ID']) + 
                                  "<br>Duration: "+ str(round(self.data ['Duration'],2)) + 
                                  "<br>SetupTime: " + str(round(self.data ['SetupTime'],2)) + "</b>")
        
    def update(self):
        #Heading
        if self.heading_val_box.text() == "":
            self.heading_val_box.setText(str(0))
            set_heading = 0
        else:
            set_heading = float(self.heading_val_box.text())
            self.heading_val_box.setText(str(set_heading))
            self.data['heading_setpoint'] = set_heading

        #Depth
        if self.depth_val_box.text() == "":
            self.depth_val_box.setText(str(0))
            set_depth = 0
        else:
            set_depth = float(self.depth_val_box.text())
            self.depth_val_box.setText(str(set_depth))
            self.data['depth_setpoint'] = set_depth
            
        #VR
        if self.VR_val_box.text() == "":
            self.VR_val_box.setText(str(0))
            set_VR = 0
        else:
            set_VR = float(self.VR_val_box.text())
            self.VR_val_box.setText(str(set_VR))
            
            
        #VL
        if self.VL_val_box.text() == "":
            self.VL_val_box.setText(str(0))
            set_VL = 0
        else:
            set_VL = float(self.VL_val_box.text())
            self.VL_val_box.setText(str(set_VL))
            
            
        #HR
        if self.HR_val_box.text() == "":
            self.HR_val_box.setText(str(0))
            set_HR = 0
        else:
            set_HR = float(self.HR_val_box.text())
            self.HR_val_box.setText(str(set_HR))
            
        
        #HL
        if self.HL_val_box.text() == "":
            self.HL_val_box.setText(str(0))
            set_HL = 0
        else:
            set_HL = float(self.HL_val_box.text())
            self.HL_val_box.setText(str(set_HL))
            
            
        #SetupTime
        if self.SetupTime_box.text() == "":
            self.SetupTime_box.setText(str(0))
            set_SetupTime = 0
        else:
            set_SetupTime = float(self.SetupTime_box.text())
            self.SetupTime_box.setText(str(set_SetupTime))
            self.data['SetupTime'] = set_SetupTime
            
        #Duration
        if self.duration_box.text() == "":
            self.duration_box.setText(str(0))
            set_duration = 0
        else:
            set_duration = float(self.duration_box.text())
            self.duration_box.setText(str(set_duration))
            self.data['Duration'] = set_duration
            
        
        self.updateConfig(set_depth, set_heading, set_VR, set_VL, set_HR, set_HL, set_SetupTime, set_duration)
            
    def engage(self):
        engage_pub = rospy.Publisher('GUI', String, queue_size=10)
        str = 'Start'
        engage_pub.publish(str)
        
    def disarm(self):
        disarm_pub = rospy.Publisher('Gui_Disarm &', String, queue_size=10)
        str = 'Gui_Disarm'
        disarm_pub.publish(str)
    
    def rosuri(self):
        set_auv_ip = None
        set_host_ip = None
        default_auv_ip = "192.168.2.1"
        default_host_ip = "192.168.2.2"
        
        #AUV IP
        if self.AUV_IP_box.text() == "":
            self.AUV_IP_box.setText(default_auv_ip) 
        else:
            set_auv_ip = str(self.AUV_IP_box.text())
            self.AUV_IP_box.setText(set_auv_ip)
            
        #HOST IP
        if self.Host_IP_box.text() == "":
            self.Host_IP_box.setText("192.168.2.2")
        else:
            set_host_ip = str(self.Host_IP_box.text())
            self.Host_IP_box.setText(default_host_ip)
        if set_auv_ip and set_host_ip != "":
                os.system ('export ROS_HOSTNAME="$set_auv_ip" &')
                os.system ('export ROS_HOSTNAME="$set_host_ip" &')
        else:
                os.system ('export ROS_HOSTNAME="$default_auv_ip" &')
                os.system ('export ROS_HOSTNAME="$default_host_ip" &')
                
    def onActivated(self,index):
        self.isFront = index
                
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_img):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        except CvBridgeError as e:
            pass

        return frame 

    def update_video_front(self,image):
        #convert numpy mat to pixmap image
        cvBGRImg_front = self.drawReticle(self.rosimg2cv(image))
        #cv2.cvtColor(self.drawReticle(self.rosimg2cv(image)), cv2.cv.CV_BGR2RGB)
        bbLock = threading.Lock()
        try:
            bbLock.acquire()
            cvRGBImg_front = cv2.cvtColor(cvBGRImg_front, cv2.COLOR_BGR2RGB)
            qimg = QImage(cvRGBImg_front.data,cvRGBImg_front.shape[1], cvRGBImg_front.shape[0], QImage.Format_RGB888)
        finally:
            bbLock.release()
        qpm = QPixmap.fromImage(qimg)
        self.video_top.setPixmap(qpm.scaledToHeight(250))

    def update_video_bot(self,image):
        cvBGRImg_bot = self.rosimg2cv(image)
        #if self.isSonar == 1:
        #    cvBGRImg_bot = cv2.resize(cvBGRImg_bot, (360, 250))
        cvRGBImg_bot = cv2.cvtColor(cvBGRImg_bot, cv2.COLOR_BGR2RGB)
        qimg = QImage(cvRGBImg_bot.data,cvRGBImg_bot.shape[1], cvRGBImg_bot.shape[0], QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.video_bot.setPixmap(qpm.scaledToHeight(250))

    def front_callback(self,image):
        try:
            self.q_image_front = image
        except CvBridgeError, e:
            print e

    def bottom_callback(self,image):
        try:
            self.q_image_bot = image
        except CvBridgeError, e:
            print e     
        
    def pub_Disarmedcallback(self, pressed):
        
        self.pubDis.publish('Disarmed')
        
    def initImage(self):
        self.bridge = CvBridge()
        self.frontcam_sub = rospy.Subscriber("/image_raw", Image, self.front_callback) 
        self.botcam_sub = rospy.Subscriber("/image_filtered", Image, self.bottom_callback)

    def initSub(self):
        self.sub_compass = rospy.Subscriber("/SpartonCompass", Float32, self.compass_callback);
        self.sub_sounder = rospy.Subscriber("/DepthSounder", Float32, self.sonar_callback);
        self.sub_taskid = rospy.Subscriber("/Task_id", String, self.taskid_callback);
        self.sub_VR = rospy.Subscriber("/VRM", Int16, self.VRM_callback);
        self.sub_sounder = rospy.Subscriber("/VLM", Int16, self.VLM_callback);
        self.sub_sounder = rospy.Subscriber("/HLM", Int16, self.HRM_callback);
        self.sub_sounder = rospy.Subscriber("/HLM", Int16, self.HLM_callback);
        
    def valueChanged(self,value):
        self.heading_val_box.setText(str(value))

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
    
    #Call backs tied to initSub
    def compass_callback(self,data):
        self.q_heading.put(data)
    def sonar_callback(self,data):
        self.q_depth.put(data)
    def taskid_callback(self,data):
        self.q_Task_id.put(data)
    def VRM_callback(self,data):
        self.q_vrm.put(data)
    def VLM_callback(self,data):
        self.q_vlm.put(data)
    def HRM_callback(self,data):
        self.q_hrm.put(data)
    def HLM_callback(self,data):
        self.q_hlm.put(data)
    
        
    def drawReticle(self, origimg):
        yaw, pitch, roll = self.data['yaw'], self.data['pitch'], self.data['roll']

        DEGREE_PIXEL_RATIO = 0.1
        H_DEGREE_PIXEL_RATIO = 0.3
        height, width, _ = origimg.shape
        colour = (0, 0, 0)
        pitch_start, pitch_end = 40, height-40
        yaw_start, yaw_end = 40, width-40

        img = origimg

        mid_x, mid_y = width/2, height/2

        # Draw indicators
        cv2.line(img, (mid_x-70, mid_y), (mid_x-50, mid_y), (0, 0, 255), 1)
        cv2.line(img, (mid_x+50, mid_y), (mid_x+70, mid_y), (0, 0, 255), 1)
        cv2.line(img, (mid_x, 33), (mid_x-5, 38), (0, 0, 255), 1)
        cv2.line(img, (mid_x, 33), (mid_x+5, 38), (0, 0, 255), 1)
        cv2.line(img, (mid_x, pitch_end+13), (mid_x-5, pitch_end+10), (0, 0, 255), 1)
        cv2.line(img, (mid_x, pitch_end+13), (mid_x+5, pitch_end+10), (0, 0, 255), 1)

        # Multiply by 10 to work in integers
        origin_pitch = int(10 * (DEGREE_PIXEL_RATIO * (mid_y-pitch_start) + pitch))
        # Round to multiple of 25 lower than this
        BASE = 25
        closest_pitch = int(BASE * round(float(origin_pitch)/BASE))
        closest_pitch -= BASE if closest_pitch > origin_pitch else 0

        pitch_y = pitch_start + int((origin_pitch - closest_pitch) / (10 * DEGREE_PIXEL_RATIO))
        pitch_inc = int(BASE / (10 * DEGREE_PIXEL_RATIO))
        current_pitch = closest_pitch

        # Draw horizontal lines
        while pitch_y < pitch_end:
            thickness = 1
            offset = 6
            if current_pitch % 50 == 0:
                offset = 10
            if current_pitch % 100 == 0:
                offset = 18

            pt1 = (mid_x-offset, pitch_y)
            pt2 = (mid_x+offset, pitch_y)
            cv2.line(img, pt1, pt2, colour, thickness)

            if current_pitch % 100 == 0:
                txt = str(abs(current_pitch)/10)
                (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 1)
                pt = (mid_x-offset-txt_w-2, pitch_y + txt_h/2)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour)

                pt = (mid_x+offset+2, pitch_y + txt_h/2)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour)

            current_pitch -= BASE
            pitch_y += pitch_inc

        # Draw arc
        angle = int(180 - roll)
        cv2.ellipse(img, (mid_x, 140), (180, 120), angle, 75, 105, colour)
        arcpts = cv2.ellipse2Poly((mid_x, 140), (180, 120), angle, 75, 105, 15)
        for i, pt in enumerate(arcpts):
            disp_angle = (i-1) * 15
            txt = str(abs(disp_angle))
            txt_angle = np.deg2rad(-roll - 90 + disp_angle)
            (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 1)
            txt_x = int(pt[0] + 6 * math.cos(txt_angle)) - txt_w/2
            txt_y = int(pt[1] + 6 * math.sin(txt_angle))

            cv2.putText(img, txt, (txt_x, txt_y), cv2.FONT_HERSHEY_PLAIN, 0.8, colour)
            cv2.ellipse(img, (pt[0], pt[1]), (1,1), 0, 0, 360, colour)

        # Draw horizontal band
        CARDINALS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']

        origin_yaw = int(-H_DEGREE_PIXEL_RATIO * (mid_x-yaw_start) + yaw)
        # Round to multiple of 5 greater than this
        H_BASE = 5
        closest_yaw = int(H_BASE * round(float(origin_yaw)/H_BASE))
        closest_yaw += H_BASE if closest_yaw < origin_yaw else 0

        yaw_x = 5 + yaw_start + int((closest_yaw - origin_yaw) / float(H_DEGREE_PIXEL_RATIO))
        yaw_inc = int(H_BASE / float(H_DEGREE_PIXEL_RATIO))
        current_yaw = closest_yaw

        yaw_bottom = pitch_end + 30

        while yaw_x < yaw_end:
            thickness = 1
            offset = 3 if current_yaw % 15 else 6

            pt1 = (yaw_x, yaw_bottom)
            pt2 = (yaw_x, yaw_bottom - offset)
            cv2.line(img, pt1, pt2, colour, thickness)

            if current_yaw % 15 == 0:
                disp_yaw = current_yaw if current_yaw >= 0 else current_yaw + 360
                disp_yaw = disp_yaw if current_yaw < 360 else current_yaw - 360
                txt = str(disp_yaw) if current_yaw % 45 else CARDINALS[disp_yaw / 45]
                (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 1)
                pt = (yaw_x-txt_w/2, yaw_bottom - txt_h)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour)

            current_yaw += H_BASE
            yaw_x += yaw_inc

        return img

    def signal_handler(self, signal, frame):
        sys.exit(0)
        
    def updateConfig(self,goal_depth,goal_heading,VR,VL,HR,HL,SetupTime,duration):
        #Update Config file parameters
        client = dynamic_reconfigure.client.Client('Mission')
        missionparams = { 'depth_setpoint' : goal_depth , 'heading_setpoint' : goal_heading , 'SetupTime' : SetupTime , 'Duration' : duration, 'VL' : VL, 'VR' : VR, 'HR' : HR, 'HL' : HL}
        config = client.update_configuration(missionparams)
        
    #===========================End of event handler=======================================================
        
if __name__ == "__main__":
    rospy.init_node('Mission_Control', anonymous=True)
    app = QApplication(sys.argv)
    form = AUV_gui()
    #signal.signal(signal.SIGINT, form.signal_handler)
    form.show()
    app.exec_()
