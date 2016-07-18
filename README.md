# bwi_pulse_detector
status: in progress

Implementation of Eulerian Video Magnification by Kathryn Baldauf and Kiana Alcala

Code to allow robot to detect the pulse of a human or amplify subtle changes in the movement of a scene. 
The code is broken into two classes and a main method (Eulerian_MotionMag.cpp).
The color_mag class amplifies the color of the scene while the motion_mag class amplifies the subtle motions of a video

# To run:
Download all files into a ros package
Make sure to update the CMakeLists.txt and package.xml with your package name if changed from bwi_pulse_detector
start a roscore if necessary

To use rosbags: 
go to the directory with your rosbags 
in the terminal type 
rosbag play -l <bag name>

Catkin_make in your terminal to ensure you have access to the imports, and nothing is wrong, etc
in terminal type:
rosrun <ros package name> evm 

# Motion mag 
To do Motion magnification, make sure to uncomment the following lines in Eulerian_MotionMag.cpp: 

  motion_mag mot;
  
  mot.motionMag();
  
# Color Mag
To do Color magnification, make sure to uncomment the following lines in Eulerian_MotionMag.cpp: 

  color_mag mag;
  
  mag.magColor();
  
#Code adapted from the following repositories:

https://github.com/wzpan/QtEVM

https://github.com/kgram007/Eulerian-Motion-Magnification

https://github.com/thearn/webcam-pulse-detector
