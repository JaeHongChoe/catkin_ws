#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.hpp>
#include <iostream>

#include "std_msgs/Int16.h"

#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <string>
//#include "camera_test/where.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main(int argc, char**argv){
  //for cam----------------------------------------------
  VideoCapture cap(0);
  Mat frame, grayframe; 

  if(!cap.isOpened()){
    cout<<"no camera"<<endl;
    return -1;
  }

  int const Xcenter = 160;//X center
  int const Yvalue = 100;//Y center
  int const width =639;
  int const range =40;
  Point p1(1,Yvalue), p2(width/2,Yvalue);//from right to middle
  Point p3(Xcenter,Yvalue-20), p4(Xcenter,Yvalue+20);// make center mark
  int Lvalue=130;
  int Lpixel=0, Rpixel=0;
  int Ldistance=275, Rdistance=320, pastL;
  int differ;
  //--------------------------------------------------
  //for msg pub --------------------------------------
  ros::init(argc, argv, "cam_msg_publisher");
  ros::NodeHandle nh;
  std_msgs::Int16 cam_msg;
  ros::Publisher pub = nh.advertise<std_msgs::Int16>("cam_msg",100);

  int init_past=1;
  int pub_value;
  //--------------------------------------------------
  ros::Rate loop_rate(50);
  cout<<"start"<<endl;
 
  //clock_t tStart = clock();
  for(;;){
	
    cap>>frame;
	if(waitKey(30) >=0 )break; 
   	resize(frame,frame,Size(320,200)); 
	imshow("frame",frame);
    cvtColor(frame,grayframe,COLOR_BGR2GRAY) ; 
	imshow("gray",grayframe); 
/*
	GaussianBlur(grayframe,grayframe, Size(9,9),0);
	imshow("GGGG", grayframe);
	*/

	//threshold 

	Mat thresframe;
	threshold(grayframe,thresframe,100,255,THRESH_BINARY);
	imshow("threshold", thresframe);
	
	/* canny
	Mat cannyframe;
	Canny(grayframe, cannyframe, 30,90,3);
	imshow("canny",cannyframe); 
	*/


	//polylines

	Mat mask = Mat::zeros(grayframe.size(), grayframe.type());
	Mat output;
	// to make interest area(points).
	cv::Point pts[5] = {
	cv::Point(0,200),
	cv::Point(0,180),
	cv::Point(70,120),
	cv::Point(220,120),
	cv::Point(320,200)
	};
	
	Scalar yy(255,255,255);
	//polylines(grayframe, &pts, &npts, 1,false, Scalar(0,255,0),2);
	fillConvexPoly(mask, pts, 5, yy); // mask
	
	//put mask on frame(video)
	Mat bitwiseframe;
	bitwise_and(thresframe, mask,bitwiseframe); 
	//addWeighted(grayframe, 1, img_edges, 0.5,0, grayframe);
	imshow("poly", bitwiseframe);

	
	//use hough lines to draw lines 
	vector<cv::Vec4i> line;
	HoughLinesP(bitwiseframe,line,1,CV_PI/180,20,20,30);

	
	for(int i=0; i< line.size();i++){
	cv::Vec4i l = line[i];
	cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2],l[3]), Scalar(0,0,255),2);
	};
	
	//put lines on original frame to get visual image.
	Mat norm;
	normalize(frame, norm, 0,255, NORM_MINMAX,CV_8U);
	imshow("houghline",norm);
	
	
/*
	//sort right and left lines (test)
	//Mat output;
	//std::vector<std::vector<cv::Vec4i> > output(2);
	std::vector<cv::Point> output1(4);
  	size_t j = 0;
  	cv::Point ini;
  	cv::Point fini;
	cv::Point ini2;
  	cv::Point fini2;
  	double slope_thresh = 0.3;
	bool right_flag, left_flag;
  	std::vector<double> slopes;
  	std::vector<cv::Vec4i> selected_lines;
  	std::vector<cv::Vec4i> right_lines, left_lines;
	std::vector<std::vector<cv::Vec4i> > left_right_lines;
	cv::Vec4d right_line;
  	cv::Vec4d left_line;
	std::vector<cv::Point> right_pts;
	std::vector<cv::Point> left_pts;
	

  	// Calculate the slope of all the detected lines
  	for(int i=0; i< line.size();i++){
	cv::Vec4i l = line[i];
	ini = cv::Point(l[0], l[1]);
    fini = cv::Point(l[2], l[3]);
    // Basic algebra: slope = (y1 - y0)/(x1 - x0)
    	double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y))/		(static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

    // If the slope is too horizontal, discard the line
    // If not, save them  and their respective slope
    	if (std::abs(slope) > slope_thresh) {
      		slopes.push_back(slope);
      		selected_lines.push_back(i);
    }
  }

  // Split the lines into right and left lines
  	double img_center = static_cast<double>((thresframe.cols / 2));
  	while (j < selected_lines.size()) {
    	ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
    	fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

    // Condition to classify line as left side or right side
    if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
      	right_lines.push_back(selected_lines[j]);
      	right_flag = true;
    } else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
        left_lines.push_back(selected_lines[j]);
        left_flag = true;
    }
    j++;
  }

  	left_right_lines[0] = right_lines;
  	left_right_lines[1] = left_lines;
	//cout << typeid(left_right_lines[0]).name();
	//left_right_lines = output;


	// If right lines are being detected, fit a line using all the init and final points of the lines
  	if (right_flag == true) 
{
    	for (std::vector<cv::Point> i : left_right_lines[0]) {
		//cv::Vec4i l = line[i];
      	ini = cv::Point(i[0], i[1]);
      	fini = cv::Point(i[2], i[3]);

      	right_pts.push_back(ini);
      	right_pts.push_back(fini);
    	}

    	if (right_pts.size() > 0) 
{
      	// The right line is formed here
      	cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
      	double right_m = right_line[1] / right_line[0];
      	cv::Point right_b = cv::Point(right_line[2], right_line[3]);
    	}
  	}

  // If left lines are being detected, fit a line using all the init and final points of the lines
  if(left_flag == true) 
{
    for (cv::Vec4i j1 : left_right_lines[1]) {
		//cv::Vec4i j1 = line[i];
      	ini2 = cv::Point(j1[0], j1[1]);
      	fini2 = cv::Point(j1[2], j1[3]);

      	left_pts.push_back(ini2);
      	left_pts.push_back(fini2);
    }

    if (left_pts.size() > 0) 
{
      // The left line is formed here
      cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
      double left_m = left_line[1] / left_line[0];
      cv::Point left_b = cv::Point(left_line[2], left_line[3]);
    }
  }

  // One the slope and offset points have been obtained, apply the line equation to obtain the line points
  int ini_y = frame.rows;
  int fin_y = 470;

  double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
  double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

  double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
  double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

  output1[0] = cv::Point(right_ini_x, ini_y);
  output1[1] = cv::Point(right_fin_x, fin_y);
  output1[2] = cv::Point(left_ini_x, ini_y);
  output1[3] = cv::Point(left_fin_x, fin_y);

	cv::line(frame, output1[0], output1[1], Scalar(0,0,255),2);
	cv::line(frame, output1[2], output1[3], Scalar(0,0,255),2);

	Mat norm;
	normalize(frame, norm, 0,255, NORM_MINMAX,CV_8U);
	imshow("houghline",norm);
*/
    loop_rate.sleep();
	//printf("time::: %.fs\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);   


  }



  cout<<"Camera off"<<endl;
  return 0;
}
