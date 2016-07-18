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
	image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, &&ColorActionServer::pose_cb, this);
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
  
  	void pose_cb(const sensor_msgs::ImageConstPtr& msg)
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
	
	void upsamplingFromGaussianPyramid(const cv::Mat &src, const int levels, cv::Mat &dst){
		Mat currentImg = src.clone();	
		cv::Mat up;
		cv::pyrUp(currentImg, up);
		currentImg = up;
		resize(currentImg, currentImg, dst.size());
		dst+=currentImg;
	}

	bool buildGaussianPyramid(const cv::Mat &img,const int levels, std::vector<cv::Mat> &pyramid)
	{
		if (levels < 1){
			ROS_INFO_STREAM("levels should be larger than 1");
			return false;
		}
		pyramid.clear();
		cv::Mat currentImg = img;
		for (int l=0; l<levels; l++) {
			cv::Mat down;
			cv::pyrDown(currentImg, down);        
			pyramid.push_back(down);
			currentImg = down;
		}
		return true;
	}

	void amplify(const Mat &src, Mat &dst)
	{		
		dst = src * alpha;	
	}
	
	void createIdealBandpassFilter(cv::Mat &filter, double fl, double fh, double rate){
		int width = filter.cols;
		int height = filter.rows;

		fl = 2 * fl * width / rate;
		fh = 2 * fh * width / rate;

		double response;

		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				// filter response
				if (j >= fl && j <= fh)
					response = 1.0f;
				else
					response = 0.0f;
				filter.at<float>(i, j) = response;
			}
		}
	}

	void temporalIdealFilter(const cv::Mat &src, cv::Mat &dst)
	{
		cv::Mat channels[3];
		// split into 3 channels
		cv::split(src, channels);

		for (int i = 0; i < 3; ++i){

			cv::Mat current = channels[i];  // current channel
			cv::Mat tempImg;

			int width = cv::getOptimalDFTSize(current.cols);
			int height = cv::getOptimalDFTSize(current.rows);

			cv::copyMakeBorder(current, tempImg, 0, height - current.rows, 0, width - current.cols,
							   cv::BORDER_CONSTANT, cv::Scalar::all(0));

			// do the DFT
			cv::dft(tempImg, tempImg, cv::DFT_ROWS | cv::DFT_SCALE);
			// construct the filter
			cv::Mat filter = tempImg.clone();
			createIdealBandpassFilter(filter, fl, fh, rate);

			// apply filter
			cv::mulSpectrums(tempImg, filter, tempImg, cv::DFT_ROWS);

			// do the inverse DFT on filtered image
			cv::idft(tempImg, tempImg, cv::DFT_ROWS | cv::DFT_SCALE);

			// copy back to the current channel
			tempImg(cv::Rect(0, 0, current.cols, current.rows)).copyTo(channels[i]);
		}
		// merge channels
		cv::merge(channels, 3, dst);

		// normalize the filtered image
		cv::normalize(dst, dst, 0, 1, CV_MINMAX);
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
		while(ros::ok()){
			ROS_INFO_STREAM("got to 1");
			cv::Mat input;
			cv::Mat output;
			cv::Mat temp;
			int f_num = 0;

			while(1){
				listen_for_data();
				
				// 1. spatial filtering
				input = cv_ptr -> image;
				if(input.empty()){
					ROS_INFO_STREAM("error");
					break;
				}

				output = input.clone();
				output.convertTo(output, CV_32FC3);
				
				cv::Mat s;
				s = output.clone();
				
				std::vector<cv::Mat> pyramid;

				buildGaussianPyramid(s, levels, pyramid);
								
				cv::Mat motion = s.clone();
				
				std::vector<cv::Mat> filtered(pyramid);
				
				
				for (int i=0; i<levels; ++i) {
					curLevel = i;
					temporalIdealFilter(pyramid.at(i), filtered.at(i));	
					amplify(filtered.at(i), filtered.at(i));	
					
					upsamplingFromGaussianPyramid(filtered.at(i), levels, motion); 
						
					//resize(filtered.at(i), filtered.at(i), motion.size());
					//temp = filtered.at(i) + motion;
					resize(motion, motion, s.size());
					temp = s+ motion;					

					
					resize(temp, temp, output.size());
					output = temp.clone();
					
					cv::Mat colored;
					colored = s.clone();
					
					
					double minVal, maxVal;
					minMaxLoc(output, &minVal, &maxVal); //find minimum and maximum intensities
					
					output.convertTo(output, CV_8UC3, 255.0/(maxVal), -minVal * 255.0/(maxVal - minVal));
					
					cout << f_num++;
					imshow(IN_WINDOW, input);
					imshow(OUT_WINDOW, output);
					char c = waitKey(1);
					if(c == 27)
						break;
				}
				//pulse /= levels;
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
