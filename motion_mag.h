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
/********************************************************************/

using namespace cv;
using namespace std;

#define OUT_WINDOW	"Output Window"
#define IN_WINDOW   "Input Window"


class motion_mag{
	/*************************  Variables  *******************************/
	//mathematical variables
	float fl;
	float fh ; //I think fl corresponds to r2 and fh corresponds to r1
	float alpha;
	float chromAttenuation; 
	float lambda_c;
	float exaggeration_factor;
	float delta;
	float lambda;
	int levels;
	int length;
	float rate;

	int curLevel;
	bool foundData;

	//cv variables 
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat orig;
	
	std::vector<cv::Mat> lowpass1;
	std::vector<cv::Mat> lowpass2;
	
	//ros variables
	ros::NodeHandle n;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	/********************************************************************/
	
public: 
	
motion_mag() : it_(n){
	image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, &motion_mag::pose_cb, this);
   	image_pub_ = it_.advertise("/image_converter/output_video", 1);
	foundData = false;
	fl = 0.05; //originally 0.05
	fh = 0.4; // originally 1 //I think fl corresponds to r2 and fh corresponds to r1
	alpha = 10; //originally 50
	chromAttenuation = 0.1; //originally 0.1 
	exaggeration_factor = 2.0; //originally 2
	delta = 0;
	lambda = 0;
	lambda_c = 16;
	levels = 5; //originally 5
	curLevel = 0;
	length = 0;
	rate = 30;
	cvNamedWindow(IN_WINDOW, CV_WINDOW_AUTOSIZE);
	cvMoveWindow(IN_WINDOW, 20, 100 ); 
	cvNamedWindow(OUT_WINDOW, CV_WINDOW_AUTOSIZE);
	cvMoveWindow(OUT_WINDOW, 20, 100 );
	
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
	while(!foundData){
		loop_rate.sleep();
		ros::spinOnce();
	}
	foundData = false;
}

Mat LaplacianPyr(Mat img){
	Mat down, up, lap;
	pyrDown(img, down);
	pyrUp(down, up);
	lap = img - up;
	return lap;
}
	

bool buildLaplacianPyramid(const Mat &img, const int levels, vector<Mat> &pyramid)
{
    if (levels < 1)
	{
        ROS_INFO_STREAM("Levels should be larger than 1");
        return false;
    }
    pyramid.clear();
    Mat currentImg = img;
    for (int l=0; l<levels; l++)
	{
        Mat down,up;
        pyrDown(currentImg, down);
        pyrUp(down, up, currentImg.size());
        Mat lap = currentImg - up;
        pyramid.push_back(lap);
        currentImg = down;
    }
    pyramid.push_back(currentImg);
    return true;
}


void reconImgFromLaplacianPyramid(const vector<Mat> &pyramid, const int levels, Mat &dst)
{
    Mat currentImg = pyramid[levels];
    for (int l=levels-1; l>=0; l--)
	{
        Mat up;
        pyrUp(currentImg, up, pyramid[l].size());
        currentImg = up + pyramid[l];
    }
    dst = currentImg.clone();
}


void temporalIIRFilter(const Mat &src, Mat &dst)
{
    cv::Mat temp1 = (1-fh)*lowpass1[curLevel] + fh*src;
    cv::Mat temp2 = (1-fl)*lowpass2[curLevel] + fl*src;
    lowpass1[curLevel] = temp1;
    lowpass2[curLevel] = temp2;
    dst = lowpass1[curLevel] - lowpass2[curLevel];
}


void amplify(const Mat &src, Mat &dst)
{
    float currAlpha;
    //compute modified alpha for this level
    currAlpha = lambda/delta/8 - 1;
    currAlpha *= exaggeration_factor;
    if (curLevel==levels || curLevel==0)     // ignore the highest and lowest frequency band
        dst = src * 0;
    else
        dst = src * cv::min(alpha, currAlpha);
}


void attenuate(Mat &src, Mat &dst)
{
    Mat planes[3];
    //split the src into three planes
    split(src, planes);
    //don't muliply the first plane by the chromeAttenuation
    planes[1] = planes[1] * chromAttenuation;
    planes[2] = planes[2] * chromAttenuation;
    merge(planes, 3, dst);
}


int motionMag(){
	ROS_INFO_STREAM("Magnifying motion...");
	while(ros::ok()){
		Mat frame;
		Mat motion;
		Mat output;
		vector<Mat> pyramid;
		vector<Mat> filtered;
		int f_num = 0;
		listen_for_data();
		while(1){
			listen_for_data();
			orig = cv_ptr -> image;
			if(orig.empty()){
				ROS_INFO_STREAM("out of data");
				break;
			}
			frame = orig.clone();
			frame.convertTo(frame, CV_32FC3, 1.0/255.0f);
			cvtColor(frame, frame, CV_BGR2Lab);

			Mat s = frame.clone();
			buildLaplacianPyramid(s, levels, pyramid);

			if(f_num == 0){
				lowpass1 = pyramid;
				lowpass2 = pyramid;
				filtered = pyramid;
			}else{
				for (int i=0; i<levels; ++i) {
					curLevel = i;
					temporalIIRFilter(pyramid.at(i), filtered.at(i));
				}

				// amplify each spatial frequency bands according to Figure 6 of paper            
				cv::Size filterSize = filtered.at(0).size();
				int w = filterSize.width;
				int h = filterSize.height;

				delta = lambda_c/8.0/(1.0+alpha);
				// the factor to boost alpha above the bound(for better visualization)
				exaggeration_factor = 2.0;

				// compute the representative wavelength lambda for the lowest spatial frequency band of Laplacian pyramid
				lambda = sqrt( (float)(w*w + h*h) ) / 3;  // 3 is experimental constant

				for (int i=levels; i>=0; i--){
					curLevel = i;
					amplify(filtered.at(i), filtered.at(i));

					// go one level down on pyramid representative lambda will reduce by factor of 2
					lambda /= 2.0;
				}
			}

			// 4. reconstruct motion image from filtered pyramid
			reconImgFromLaplacianPyramid(filtered, levels, motion);

			// 5. attenuate I, Q channels
			attenuate(motion, motion);

			// 6. combine source frame and motion image
			if (f_num > 0)    // don't amplify first frame
				s += motion;

			// 7. convert back to rgb color space and CV_8UC3
			output = s.clone();
			cvtColor(output, output, CV_Lab2BGR);
			output.convertTo(output, CV_8UC3, 255.0, 1.0/255.0);

			imshow(OUT_WINDOW, output);
			imshow(IN_WINDOW, orig);
			f_num++;
		
			char c = waitKey(1);
			if(c == 27)
				break;
				
			}
			ros::spin();
		}
	}

};
