#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include <spauv/MissionConfig.h>
using namespace std;

//Global variables
int state;
float VL,VR,HL,HR;
float depth_setpoint,heading_setpoint;
float yaw_feedback,sonar_feedback;
double SetupTime,Duration,time_since_go;
double original_yaw,original_depth;
std_msgs::Float32 goal_heading,goal_depth;
std_msgs::Int16 VL_GUI,VR_GUI,HR_GUI,HL_GUI;
std_msgs::String Task_id;
string gui_button,op_mode,depth_controller,forward_controller,id;

void callback(spauv::MissionConfig &config, uint32_t level) {
  ROS_INFO("Mission Reconfigure Request: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", 
            depth_setpoint=config.depth_setpoint, heading_setpoint=config.heading_setpoint, 
            SetupTime=config.SetupTime, 
            Duration=config.Duration, 
            VL=config.VL, VR=config.VR,
            HL=config.HL, HR=config.HR);}

void GuiCallback(const std_msgs::String::ConstPtr& msg){gui_button = msg->data;}
void OpModeCallback(const std_msgs::String::ConstPtr& msg){op_mode = msg->data;}
void depthcontroller_Callback(const std_msgs::String::ConstPtr& msg){depth_controller = msg->data;}
void fwdcontroller_Callback(const std_msgs::String::ConstPtr& msg){forward_controller = msg->data;}
void CompassCallback(const geometry_msgs::Pose2D::ConstPtr& msg){yaw_feedback = msg->theta;}
void SonarCallback(const std_msgs::Float32::ConstPtr& msg){sonar_feedback = msg->data;}
void TimerCallback(const ros::TimerEvent& event){time_since_go += 0.1; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "Mission");

  dynamic_reconfigure::Server<spauv::MissionConfig> server;
  dynamic_reconfigure::Server<spauv::MissionConfig>::CallbackType f;
  
  ros::NodeHandle n;
  //Time
  ros::Timer timer = n.createTimer(ros::Duration(0.1), TimerCallback);
  //Subscribers
  ros::Subscriber sub_gui = n.subscribe("GUI", 1000, GuiCallback);
  ros::Subscriber sub_omode = n.subscribe("OpMode", 1000, OpModeCallback);
  ros::Subscriber sub_depth_controller = n.subscribe("depth_controller", 1000, depthcontroller_Callback);
  ros::Subscriber sub_forward_controller = n.subscribe("forward_controller", 1000, fwdcontroller_Callback);
  ros::Subscriber sub_compass = n.subscribe("SpartonCompass", 1000, CompassCallback);
  ros::Subscriber sub_sonar = n.subscribe("DepthSounder", 1000, SonarCallback);
  //Publishers
  ros::Publisher pub_goal_depth = n.advertise<std_msgs::Float32>("depth_setpoint", 500);
  ros::Publisher pub_goal_heading = n.advertise<std_msgs::Float32>("heading_setpoint", 500);
  ros::Publisher pub_VR_GUI = n.advertise<std_msgs::Int16>("VR_GUI", 500);
  ros::Publisher pub_VL_GUI = n.advertise<std_msgs::Int16>("VL_GUI", 500);
  ros::Publisher pub_HR_GUI = n.advertise<std_msgs::Int16>("HR_GUI", 500);
  ros::Publisher pub_HL_GUI = n.advertise<std_msgs::Int16>("HL_GUI", 500);
  ros::Publisher pub_Task_id = n.advertise<std_msgs::String>("Task_id", 500);

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ros::Rate loop_rate(5);

  //Variables :
  state = 1;

  while (ros::ok()) {

	ros::spinOnce();
	if( gui_button == "Start"){
	switch (state){
		case 1:
        		time_since_go = 0;
        		//Securing Initial Position
			if(op_mode=="QUAL" || op_mode=="FINAL"){
          		original_yaw = yaw_feedback;
			goal_heading.data = heading_setpoint;
			original_depth = sonar_feedback;
			goal_depth.data = depth_setpoint;
			id = "/Initialising";
          		state = 2;}
		break;

		case 2:
			//AUV GOING DOWN
			if ((int)time_since_go > 2 ){
          		VL_GUI.data = VL;
          		VR_GUI.data = VR;
			HL_GUI.data = 0;
			HR_GUI.data = 0; 
			id = "/Descent";
			if(depth_controller=="Succeeded") {state = 3;}}

     		break;

      		case 3:
			//AUV GOING STRAIGHT
			VL_GUI.data = VL;
          		VR_GUI.data = VR;
			HL_GUI.data = HL;
			HR_GUI.data = HR;
			id = "/Forward";
			if(forward_controller=="Succeeded") {state = 4;}
		break;
		
		case 4:
			//GATE DETECTION
			Task_id.data = 3;
			return(0);
		break; }}

    Task_id.data = id;
    pub_goal_heading.publish(goal_heading);
    pub_goal_depth.publish(goal_depth);
    pub_VL_GUI.publish(VL_GUI);
    pub_VR_GUI.publish(VR_GUI);
    pub_HL_GUI.publish(HL_GUI);
    pub_HR_GUI.publish(HR_GUI);
    pub_Task_id.publish(Task_id);
	
    loop_rate.sleep(); //End of main loop

  }

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

