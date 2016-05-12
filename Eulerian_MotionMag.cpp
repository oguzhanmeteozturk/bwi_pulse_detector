//////////////****  Eulerian Motion Magnification  ****////////////////
//
//	File Name:	 Eulerian_MotionMag.cpp
//	Author:		 Ramsundar K G
//	Date:		 8 July 2015
//
//	Description: This is The C++ implementation of the following paper
//				 using the OpenCV library: "Eulerian Video Magnification
//				 for Revealing Subtle Changes in the World"
//
//////////////////////////////////////////////////////////////////////


/*************************  Includes  *******************************/
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <stdio.h>
#include <iostream>

#include "color_mag.h"
#include "motion_mag.h"

#include <vector>
#include <math.h>
#include <list>
#include "cv.h"			// Computer Vision Lib Header
#include "highgui.h"	// HighGUI Vision Lib Header
/********************************************************************/

using namespace cv;
using namespace std;

#define OUT_WINDOW	"Output Window"
#define IN_WINDOW   "Input Window"


int main(int argc, char** argv)
{
	ROS_INFO_STREAM("Hello");
	ros::init(argc, argv, "evm");
	
	color_mag mag;
	mag.magColor();
		
	//motion_mag mot;
	//mot.motionMag();

	return 0;
}

