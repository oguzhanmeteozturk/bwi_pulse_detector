/*************************  Includes  *******************************/
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <stdio.h>
#include <iostream>

#include <vector>
#include <math.h>
#include <list>
#include "cv.h"			// Computer Vision Lib Header
#include "highgui.h"	// HighGUI Vision Lib Header

#include <bwi_pulse_detector/ColorAction.h>
/********************************************************************/

using namespace cv;
using namespace std;

#define OUT_WINDOW	"Output Window"
#define IN_WINDOW   "Input Window"


class ColorActionServer
{
protected:

  ros::NodeHandle nh_;
  
  actionlib::SimpleActionServer<bwi_pulse_detector::ColorAction> as_; 
  
  std::string action_name_;
  
  bwi_pulse_detector::ColorFeedback feedback_;
  bwi_pulse_detector::ColorResult result_;

  float fl;
  float fh;
  float alpha;
  float chromAttenuation; 
  float exaggeration_factor;
  int levels;
  float rate;

  int curLevel;
  bool foundData;
  
  //cv variables 
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat orig;
  

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 
public:

  ColorActionServer(std::string name) :
    as_(nh_, name, boost::bind(&ColorActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
	//subscribe to kinect video
	image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, &&ColorActionServer::image_cb, this);
	//advertise results
	image_pub_ = it_.advertise("/image_converter/output_video", 1);
	
	foundData = false;
	
	//default values
	fl = 50/60;
	fh = 1; 
	alpha = 50; 
	chromAttenuation = 0.1;
	exaggeration_factor = 5.0;
	levels = 6;
	curLevel = 0;
	rate = 30;
	
	//create windows for input and output
	cvNamedWindow(IN_WINDOW, CV_WINDOW_AUTOSIZE);
	cvMoveWindow(IN_WINDOW, 20, 100 ); 
	cvNamedWindow(OUT_WINDOW, CV_WINDOW_AUTOSIZE);
	cvMoveWindow(OUT_WINDOW, 20, 100 );

	ROS_INFO("Color mag action has started");
	
    as_.start();
  }

  ~ColorActionServer(void)
  {
  }
  
  	void image_cb(const sensor_msgs::ImageConstPtr& msg)
	  {
		//converts ROS video into OpenCV format 
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  orig = cv_ptr -> image;
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
		foundData = true;
	}


	void listen_for_data(){
		ros::Rate loop_rate(40.0);
		foundData = false;
		while(!foundData){
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
	
	
	void executeCB(const bwi_pulse_detector::ColorGoalConstPtr  &goal){
		
		if (as_.isPreemptRequested() || !ros::ok()){
			ROS_INFO("Color mag action: Preempted");
			// set the action state to preempted
			as_.setPreempted();
			result_.success = false;
			as_.setSucceeded(result_);
			return;
        }
        
        //TO DO: set goal values to actual values, record rosbags
        
		ROS_INFO_STREAM("Magnifying color...");
		while(ros::ok() && !as_.isPreemptRequested()){
			//get each image in a video sequence
			
			//create laplacian or guassian pyramid
			
			//pass each level through a temporal ideal filter
			
			//multiple each level by the amplitude
			
			//add each level to original image's level
			
			//reconstruct pyramid
			
			while(1){
				
			}

			ros::spin();

		}

		result_.success = true;
		as_.setSucceeded(result_);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_mag_as");

  ColorActionServer as(ros::this_node::getName());
  ros::spin();

  return 0;
}
