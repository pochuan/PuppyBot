
#include <iostream>
#include "RobotCom.h"
#include "PrVector.h"

#include <opencv/cv.h>
#include <opencv/highgui.h> 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <stdio.h>
#include <math.h>
#include <unistd.h>

using namespace cv;

int Hue = 40;
int Sat = 150;
int Val = 150;
float RobotPos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float YThresh[] = {50, 80, 100, 200, 400, 800};
int threshold = 50;

void RotateRobot(RobotCom* Robot, double DegCCRot);
IplImage* GetThresholdedImage(IplImage* img, CvScalar minHSV, CvScalar maxHSV);
bool findBallPos(CvCapture* capture, int* lastX, int* lastY );
void CenterRobotOnBall(RobotCom* robot, CvCapture* capture);
bool ballInWorkspace(CvCapture* capture);

bool findBallPos(CvCapture* capture, int& posX, int& posY) {
  IplImage* frame = cvQueryFrame( capture );

  if ( !frame ) {
    fprintf( stderr, ": Could not retrieve frame.\n" );
    getchar();
    return false;
  }

  // printf("Hue: %d Saturation: %d Value: %d \n", Hue, Sat, Val);

  CvScalar minHSV = cvScalar(Hue, Sat, Val);
  CvScalar maxHSV = cvScalar(Hue+20, 255, 255);

  IplImage* imgThresh = GetThresholdedImage(frame, minHSV, maxHSV);

  // Calculate the moments to estimate the position of the ball
  CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
  cvMoments(imgThresh, moments, 1);

  double moment10 = cvGetSpatialMoment(moments, 1, 0);
  double moment01 = cvGetSpatialMoment(moments, 0, 1);
  double area = cvGetCentralMoment(moments, 0, 0);

  posX = moment10/area;
  posY = moment01/area;

  cvShowImage("thresh", imgThresh);
  //cvShowImage( "video", frame );
  cvReleaseImage(&imgThresh);
  //cvReleaseImage(&frame);
  delete moments;

  // Draw a line only if positions are valid
  if (area > 250 && posX > 0 && posY > 0) return true;
  
  return false;
}

IplImage* GetThresholdedImage(IplImage* img, CvScalar minHSV, CvScalar maxHSV) {
	// Convert img to HSV space
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
	cvInRangeS(imgHSV,minHSV, maxHSV, imgThreshed);
	cvReleaseImage(&imgHSV);
	return imgThreshed;
}

void RotateRobot(RobotCom* robot, double degCCRot) {
  double radians = degCCRot*3.14159 / 180.0;
  printf("Rotate robot to %f\n", radians + RobotPos[2]);
  RobotPos[2] += radians;
  robot->control(JTRACK, RobotPos, 8);
  sleep(2);
}

void CenterRobotOnBall(RobotCom* robot, CvCapture* capture) {
  int posX, posY;
  double width = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);

  while (true){
    if (findBallPos(capture, posX, posY)){
      if (posX < (width/3.0)) {
        RotateRobot(robot, -2.0);
      } else if (posX > (2.0*width/3.0)) {
        RotateRobot(robot, 2.0);
      } else {
        // ball in the center!
        printf ("Ball is center\n");
        break;
      }
    } else {
      // if ball not found, rotate 5 deg cc
      RotateRobot(robot, 5.0);
    }
  }
}

void RobotArmDown(RobotCom* robot) {
  printf("Rotate robot arm\n");
  RobotPos[4] = 90*3.14159/180.0;
  robot->control(JTRACK, RobotPos, 8);
  sleep(2);
}

void MoveForward(RobotCom* robot, double dist) {
  printf("Moving robot forward\n");
  RobotPos[0] = RobotPos[0] + dist;
  robot->control(JTRACK,RobotPos, 8);
  sleep(2);
}

void MoveForwardToBall(RobotCom* robot, CvCapture* capture) {
  while (!ballInWorkspace(capture)) {
    MoveForward(robot, 0.1);
    CenterRobotOnBall(robot, capture);
  }
}

bool ballInWorkspace(CvCapture* capture) {
  int posX, posY;
  findBallPos(capture, posX, posY);
  // Check for ball to be in workspace
  return true;

}

void FloatAndReportJointAngles(RobotCom* robot) {
  robot->control(FLOAT,RobotPos, 8);
  printf("Float\n");
  while(true) {
    float data_in[8];
 printf("Almost get status\n");
    robot->getStatus(GET_JPOS, data_in);
  printf("Get status\n");
    std::cout<<"Joint angles are "<<data_in[0]<<" "<<data_in[1]<<" "<<data_in[2]<<" "<<data_in[3]<<" "<<data_in[4]<<" "<<data_in[5]<<" "<<data_in[6]<<" "<<data_in[7]<<std::endl;
    sleep(1);
  }
}

int main(int argc, char** argv)
{
  std::cout<<"This program tests the network connectivity"<<std::endl;
  std::cout<<"between the client simulator and the servo server"<<std::endl;

  /****************************************/
  //TEST 1: ESTABLISH CONNECTION
  /****************************************/
  RobotCom *robot = new RobotCom();

  if(!robot) {
    std::cout<< "Robot not initialized"<<std::endl;
    exit(0);
  }

  // initialize camera + window screens
  CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
  if ( !capture ) {
    fprintf( stderr, "ERROR: Could not initialize capturing. \n" );
    getchar();
    exit(0);
  }

/*  cvNamedWindow("video", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("thresh", CV_WINDOW_AUTOSIZE);

  cvCreateTrackbar("Hue","thresh", &Hue, 245, NULL);
  cvCreateTrackbar("Saturation","thresh", &Sat, 245, NULL);
  cvCreateTrackbar("Value","thresh", &Val, 245, NULL);
*/
  std::cout<<"Sucessfully connected!"<<std::endl;

  /****************************************/
  //TEST 2: SEND MESSAGE TO THE SERVO SERVER
  /***************************************
  */
    FloatAndReportJointAngles(robot);
//  CenterRobotOnBall(robot, capture);
//  RobotArmDown(robot);
//  MoveForwardToBall(robot, capture);
//  cvReleaseCapture( &capture );
//  cvDestroyWindow("video"); // destroys all windows
//  cvDestroyWindow("thresh"); // destroys all windows

  /****************************************/
  //TEST 3: RECEIVE REPLIES FROM THE SERVO SERVER
  /****************************************/
  float data_in[8];
  robot->getStatus(GET_JPOS, data_in);
  std::cout<<"Joint angles are "<<data_in[0]<<" "<<data_in[1]<<" "<<data_in[2]<<" "<<data_in[3]<<" "<<data_in[4]<<" "<<data_in[5]<<" "<<data_in[6]<<" "<<data_in[7]<<std::endl;

  /****************************************/
  //TEST 4: TEAR DOWN
  /****************************************/
  delete robot;
  std::cout<<"Successfully disconnected!"<<std::endl;
}
