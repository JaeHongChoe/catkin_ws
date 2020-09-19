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

#include <time.h>
//#include "camera_test/where.h"

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
  //ros::Publisher pub("cam_msg", &cam_msg);
  //nh.advertise(pub);
  
  //ros::Publisher pub = nh.advertise<camera_test::where>("cam_msg",100);
  //camera_test::where cam_msg;
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
    cvtColor(frame,grayframe,COLOR_BGR2GRAY) ; 
    //imshow("frame",frame2);
    adaptiveThreshold(grayframe,grayframe,255,ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY_INV,21,5);
    medianBlur(grayframe,grayframe,11);
    //medianBlur(grayframe,grayframe,9);
    //medianBlur(grayframe,grayframe,5);




    line(frame,p1,p2,Scalar(100,100,255),3);
    line(frame,p3,p4,Scalar(100,100,255),3);
    

	//imshow("frame",frame);
    //imshow("lane_detection",grayframe);

    for(int c1=Xcenter;c1<639;c1++){
      Rpixel = grayframe.at<uchar>(Yvalue,c1);
      if(Rpixel == 255){
        Rdistance = c1-Xcenter;
        break;
      }

    }
    //cout<<Rdistance<<endl;
    for(int c2=Xcenter;c2>0;c2--){
      Lpixel = grayframe.at<uchar>(Yvalue,c2);
      if(Lpixel == 255){
        Ldistance = Xcenter-c2;
		if(init_past==1){
			pastL=Ldistance;
			init_past=0;
		}
        break;
      }

    }
    differ = Ldistance - Rdistance; // detect error
	cam_msg.data=differ;
	
    putText(frame,format("differ:%d", differ),Point(10,30),FONT_HERSHEY_SIMPLEX,1,Scalar(0,200,200),4);
    imshow("grayframe",grayframe);
	imshow("frame",frame);

	//exception process
	//cout<<Ldistance<<"//"<<Lvalue<<endl;	
	//if(abs(pastL-Ldistance)<40)Lvalue = Ldistance;
	
	
	//make a dicision
	
 
	/*
    if(abs(differ)<15){//forward
	  printf("abs(differ):   F      %d\n", abs(differ));
      cam_msg.data=4;
    }
    else if(30>differ&&differ>=15){//weak left
	  printf("differ:     WL    %d\n", differ);
      cam_msg.data=3;
    }
    else if(45>differ&&differ>=30){//mid left
      printf("differ:     ML    %d\n", differ);
      cam_msg.data=2;
    }
 	else if(differ>=45){//strong left
	  printf("differ:      SL   %d\n", differ);
      cam_msg.data=1;
    }
	else if(-30<differ&&differ<=-15){//weak right
      printf("differ:     WR    %d\n", differ);
      cam_msg.data=5;
    }
	else if(-45<differ&&differ<=-30){//mid right
      printf("differ:     MR    %d\n", differ);
      cam_msg.data=6;
    }
 	else if(-45>differ){//strong right
      printf("differ:      SR   %d\n", differ);
      cam_msg.data=7;
    }
    ROS_INFO("send msg = %d\n", cam_msg.data);
    */
    pub.publish(cam_msg);
	
    loop_rate.sleep();
	//printf("time::: %.fs\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);   
  }



  cout<<"Camera off"<<endl;
  return 0;
}
