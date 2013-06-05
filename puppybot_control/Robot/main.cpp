
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
int YCritThresh = 50;

void RotateRobot(RobotCom* Robot, double DegCCRot);
IplImage* GetThresholdedImage(IplImage* img, CvScalar minHSV, CvScalar maxHSV);
bool findBallPos(CvCapture* capture, int* lastX, int* lastY );
void CenterRobotOnBall(RobotCom* robot, CvCapture* capture);
bool ballInWorkspace(CvCapture* capture);

void RobotPositionReached(RobotCom* robot) {
  float pos[8];
  int count = 0;
  while(true) {
    bool reached = true;
    robot->getStatus(GET_JPOS, pos);
    for (int i=0; i <2; i++){
      float diff = pos[i] - RobotPos[i];
      if (fabs(diff) >= 0.05) {
        printf("fail: %i %f\n", i, diff);
        reached = false;
      }
    }
    float diff = (pos[2]) - RobotPos[2];
    if (fabs(diff) >= 0.003) {
      reached = false;
    }
    for (int i=3; i <8; i++){
      float diff = pos[i] - RobotPos[i];
      if (fabs(diff) >= 0.05) {
        printf("fail: %i %f\n", i, diff);
        reached = false;
      }
    }

    if (reached) break;
    //printf("Position not reached %i\n", count);
  }
  sleep(1);
}

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
  float radians = ((float)degCCRot)*3.14159 / 180.0;
  printf("Rotate robot to %f\n", degCCRot);
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  pos[2] += radians;

  for(int i=0; i<8; i++) {
    RobotPos[i] = pos[i];
  }

  robot->control(JTRACK, pos, 8);
  RobotPositionReached(robot);
}

void CenterRobotOnBall(RobotCom* robot, CvCapture* capture) {
  int posX, posY;
  double width = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);

  while (true){
    if (findBallPos(capture, posX, posY)){
      if (posX < (width/3.0)) {
        RotateRobot(robot, -1.0);
      } else if (posX > (2.0*width/3.0)) {
        RotateRobot(robot, 1.0);
      } else {
        // ball in the center!
        printf ("Ball is center\n");
        break;
      }
    } else {
      // if ball not found, rotate 5 deg cc
      RotateRobot(robot, 3.0);
    }
  }
}

void RobotArmDown(RobotCom* robot) {
  printf("Rotate robot arm\n");
  RobotPos[4] = 1.63;
  RobotPos[6] = 1.63;
  robot->control(JTRACK, RobotPos, 8);
  RobotPositionReached(robot);
}

void MoveForward(RobotCom* robot, double dist) {
  if (dist == 0) return;

  printf("Moving robot forward %f\n", dist);
  //printf("   w %f %f\n", dist*cos(RobotPos[2]), dist*sin(RobotPos[2]));
  //RobotPos[0] = RobotPos[0] + dist;//*cos(RobotPos[2]);
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  pos[0] += ((float)dist)*cosf(pos[2]);
  pos[1] += ((float)dist)*sinf(pos[2]);

  for(int i=0; i<8; i++) {
    RobotPos[i] = pos[i];
  }

  //RobotPos[2] = 0.0;
  //RobotPos[1] = RobotPos[1] + dist*sin(RobotPos[2]);
  robot->control(JTRACK,pos, 8);
  RobotPositionReached(robot);
}

void MoveForwardToBall(RobotCom* robot, CvCapture* capture) {
  int posX, posY;
  printf("move Forward To Ball\n");
  while (!ballInWorkspace(capture)) {
    findBallPos(capture, posX, posY);
    float dist = 30*0.0254;
    printf("Current posY: %f\n", posY);
    if (posY < 700) {
      float cube = posY*posY*posY;
      float sqr = posY*posY;
      dist = 0.00000024646*cube - 0.00021717*sqr + 0.073435*posY + 0.47217;
      dist = 0.0254*dist;
    } 
    dist = 3.0*dist / 4.0;
    printf("Move Forward %f\n", dist);
    MoveForward(robot, dist);
    //CenterRobotOnBall(robot, capture);

  }
}

bool ballInWorkspace(CvCapture* capture) {
  int posX, posY;
  findBallPos(capture, posX, posY);
  // Check for ball to be in workspace
  if (posY < 250) {
    return true;
  }
  return false;
}

void FloatAndReportJointAngles(RobotCom* robot) {
  robot->control(FLOAT,RobotPos, 8);
  printf("Float\n");
  while(true) {
    float data_in[8];
    robot->getStatus(GET_JPOS, data_in);
    std::cout<<"J:"<<data_in[0]<<" "<<data_in[1]<<std::endl;
    std::cout<<data_in[2]<<" "<<data_in[3]<<std::endl;
    std::cout<<data_in[4]<<" "<<data_in[5]<<std::endl;
    std::cout<<data_in[6]<<" "<<data_in[7]<<std::endl;
    sleep(2);
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
  CvCapture* capture = cvCaptureFromCAM(1);
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
//    FloatAndReportJointAngles(robot);
 // RotateRobot(robot, 30);
 // MoveForward(robot,2);
  CenterRobotOnBall(robot, capture);
  MoveForwardToBall(robot, capture);
//  RobotArmDown(robot);

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
