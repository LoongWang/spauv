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
#include <spauv/PIDConfig.h>

using namespace std;

int depth_inertia,V_PASS,H_PASS;
int V_GUI,VM,H_GUI,HM,VR_GUI,VL_GUI,HR_GUI,HL_GUI;
float user_setpoint,depth_motor_val;
float yaw_feedback,sonar_feedback,goal_depth,goal_heading;
float depth_err,depth_out,depth_base,depth_startup_time,depth_motor_kp,depth_kp;
float heading_err,heading_out,heading_base,heading_startup_time,heading_motor_kp,heading_kp;
std_msgs::Int16 VLM,VRM,HRM,HLM;
std_msgs::String dct_str,fct_str;
string dstr,fstr;

void callback(spauv::PIDConfig &config, uint32_t level) {
  ROS_INFO("Mission Reconfigure Request: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", 
            user_setpoint=config.goal, 
            depth_kp=config.depth_kp, 
            heading_kp=config.heading_kp, 
            depth_motor_kp=config.depth_motor_kp, 
            heading_motor_kp=config.heading_motor_kp,
            depth_startup_time=config.depth_startup_time,
	    heading_startup_time=config.heading_startup_time);}

void CompassCallback(const geometry_msgs::Pose2D::ConstPtr& msg){yaw_feedback = msg->theta;}
void SonarCallback(const std_msgs::Float32::ConstPtr& msg){sonar_feedback = msg->data;}
void depthsetpointCallback(const std_msgs::Float32::ConstPtr& msg){goal_depth = msg->data;}
void headingsetpointCallback(const std_msgs::Float32::ConstPtr& msg){goal_heading = msg->data;}
void VRCallback(const std_msgs::Int16::ConstPtr& msg){VR_GUI = msg->data;}
void VLCallback(const std_msgs::Int16::ConstPtr& msg){VL_GUI = msg->data;}
void HRCallback(const std_msgs::Int16::ConstPtr& msg){HR_GUI = msg->data;}
void HLCallback(const std_msgs::Int16::ConstPtr& msg){HL_GUI = msg->data;}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  dynamic_reconfigure::Server<spauv::PIDConfig> server;
  dynamic_reconfigure::Server<spauv::PIDConfig>::CallbackType f;

  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  
  ros::Subscriber sub_goal_depth = n.subscribe("depth_setpoint", 1000, depthsetpointCallback);
  ros::Subscriber sub_goal_heading = n.subscribe("heading_setpoint", 1000, headingsetpointCallback);
  ros::Subscriber sub_compass = n.subscribe("SpartonCompass", 1000, CompassCallback);
  ros::Subscriber sub_sonar = n.subscribe("DepthSounder", 1000, SonarCallback);
  ros::Subscriber sub_VR = n.subscribe("VR_GUI", 1000, VRCallback);
  ros::Subscriber sub_VL = n.subscribe("VL_GUI", 1000, VLCallback);
  ros::Subscriber sub_HR = n.subscribe("HR_GUI", 1000, HRCallback);
  ros::Subscriber sub_HL = n.subscribe("HL_GUI", 1000, HLCallback);
  ros::Publisher pub_VRM = n.advertise<std_msgs::Int16>("VRM", 500);
  ros::Publisher pub_VLM = n.advertise<std_msgs::Int16>("VLM", 500);
  ros::Publisher pub_HRM = n.advertise<std_msgs::Int16>("HRM", 500);
  ros::Publisher pub_HLM = n.advertise<std_msgs::Int16>("HLM", 500);
  ros::Publisher pub_depth_controller = n.advertise<std_msgs::String>("depth_controller", 500);
  ros::Publisher pub_forward_controller = n.advertise<std_msgs::String>("forward_controller", 500);
  VLM.data = 0;
  VRM.data = 0;
  HLM.data = 0;
  HRM.data = 0;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  while (ros::ok()) {
  	ros::spinOnce();
	
 	if (VR_GUI == VL_GUI && VRM.data == VLM.data) { V_GUI = VR_GUI; VM = VRM.data; V_PASS = 1; depth_motor_val = VRM.data;} 
        if (HR_GUI == HL_GUI && HRM.data == HLM.data) { H_GUI = HR_GUI; HM = HRM.data; H_PASS = 1;} 
	
	//if ( V_PASS == 1 && H_PASS == 1){
	//Depth Controller Block
	if (goal_depth != 0.0){
				depth_err = sonar_feedback - goal_depth;
	if (goal_depth > 0 && depth_err < 0.2){
        			if (VM < V_GUI && V_GUI > 0 || VM > V_GUI &&  V_GUI < 0 ){
                		depth_out = depth_base + (depth_startup_time + depth_motor_kp);
               			depth_base = depth_out;}
                		
            			else{ 
                		depth_out = ((depth_err * depth_kp) + (depth_motor_val));}}
	else {dstr = "Succeeded";}}
        //else {ROS_INFO("Invalid depth value. Check depth sensor");}
        //VRM.data = VLM.data = depth_out;
        VRM.data = VLM.data = VL_GUI;
	dct_str.data = dstr;        
	pub_VLM.publish(VLM);
    	pub_VRM.publish(VRM);
        pub_depth_controller.publish(dct_str);
    
        
	//Heading Controller Block
        heading_err = yaw_feedback - goal_heading;
        if (heading_err < 0.2){
        if (HM < H_GUI && H_GUI > 0 || HM > H_GUI && H_GUI < 0){
            			heading_out = heading_base + (heading_startup_time + heading_motor_kp);
            			heading_base = heading_out;
            			HRM.data = HLM.data = heading_out;}
        else{ 
            	HRM.data = ((HR_GUI) - (heading_err * heading_kp));
            	HLM.data = ((HL_GUI) + (heading_err * heading_kp));}}
	else {fstr ="Succeeded";}
	fct_str.data = fstr;
        pub_HLM.publish(HLM);
    	pub_HRM.publish(HRM);
        pub_forward_controller.publish(fct_str); //}
      

	loop_rate.sleep();}

ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

