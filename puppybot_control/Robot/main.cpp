
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
RobotCom* robot;
CvCapture* capture;

bool ballInWorkspace();
void MoveForwardToBall();
void CenterRobotOnBall(bool prec);
void RotateRobot(double DegCCRot);
bool findBallPos(int* lastX, int* lastY);
IplImage* GetThresholdedImage(IplImage* img, CvScalar minHSV, CvScalar maxHSV);

bool findBallPos(int& posX, int& posY) {
  IplImage* frame = cvQueryFrame( capture );

  if ( !frame ) {
    fprintf( stderr, ": Could not retrieve frame.\n" );
    getchar();
    return false;
  }

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
  cvReleaseImage(&imgThresh);
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

void RotateRobot(double degCCRot) {
  float radians = ((float)degCCRot)*3.14159 / 180.0;
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = radians;

  robot->control(JTRACK, pos, 8);
}

void CenterRobotOnBall(bool prec) {
  int posX, posY;
  double width = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
  
  double wl = 0;
  double wr = 0;
  if (prec) {
    wl = 3.0*width / 7.0;
    wr = 4.0*width / 7.0;
  } else {
    wl = width / 3.0;
    wr = width * 2.0/3.0;
  }

  while (true){
    if (findBallPos(posX, posY)){
      if (posX < wl) {
        RotateRobot(-1.0);
      } else if (posX > wr) {
        RotateRobot(1.0);
      } else {
        // ball in the center!
        break;
      }
    } else {
      // if ball not found, rotate 5 deg cc
      RotateRobot(3.0);
    }
  }
}

void MoveForward(double dist) {
  if (dist == 0) return;

  float pos[8];
  robot->getStatus(GET_JPOS, pos);

  pos[0] = (float) dist;
  pos[1] = 0.0;
  pos[2] = 0.0;

  robot->control(JTRACK,pos, 8);
}

void MoveForwardToBall() {
  int posX, posY;
  while (!ballInWorkspace()) {
    findBallPos(posX, posY);
    float dist = 30*0.0254;
    if (posY > 650) {
      float cube = posY*posY*posY;
      float sqr = posY*posY;
      dist = 0.00000024646*cube - 0.00021717*sqr + 0.073435*posY + 0.47217;
      dist = 0.0254*dist;
    } 
    dist = dist / 10.0;
    MoveForward(dist);
    CenterRobotOnBall(false);
  }
}

bool ballInWorkspace() {
  int posX, posY;
  findBallPos(posX, posY);
  // Check for ball to be in workspace
  if (posY < 680 && posY > 500) {
    return true;
  }
  return false;
}

void PickUpBall() {
  robot->controlGripper( (float)10 );
  
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = 0.0;

  pos[4] = 0.5;
  robot->control(JTRACK, pos, 8);
  sleep(1.5);
  pos[3] = 6*3.14159/180.0;
  pos[4] = 1.60;
  pos[6] = 1.3;
  robot->control(JTRACK, pos, 8);
  sleep(3);
  
  robot->controlGripper( (float)-5 );
  sleep(5);

  pos[7] = 1.57;
  pos[6] = 0.0;
  robot->control(JTRACK, pos, 8);
  sleep(3);
}

void trackBall() {
    int frameWidth = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    int frameCenterX = frameWidth/2;

    int posX, posY;
    while(true) {
        bool onScreen = findBallPos(posX, posY);
        if (posX < 0 || posY < 0) {
            continue;
        }
        
        bool d1 = false;
        bool d2 = false;
        if (!onScreen) {
            MoveForward(0.0);
            RotateRobot(5.0);
        } else {
            if (posY > 1000) {
                MoveForward(0.6);
            } else if (posY > 700) {
                MoveForward(0.05);
            } else if (posY < 680) {
                MoveForward(-0.05);
            } else {
                MoveForward(0.0);
                d1 = true;
            }
            
            if (posX > (frameCenterX + 100)) {
                RotateRobot(5.0);
            } else if (posX < (frameCenterX - 100)) {
                RotateRobot(-5.0);
            } else {
                RotateRobot(0.0);
                d2 = true;
            }
        }
        
        if (d1 && d2) {
            break;
        }
        
    }
    CenterRobotOnBall(true);
    PickUpBall();
    RotateRobot(20);
    sleep(0.5);
    RotateRobot(0);
}

int main(int argc, char** argv)
{
  std::cout<<"This program tests the network connectivity"<<std::endl;
  std::cout<<"between the client simulator and the servo server"<<std::endl;

  /****************************************/
  //TEST 1: ESTABLISH CONNECTION
  /****************************************/
  robot = new RobotCom();

  if(!robot) {
    std::cout<< "Robot not initialized"<<std::endl;
    exit(0);
  }

  // initialize camera + window screens
  capture = cvCaptureFromCAM(0);
  if ( !capture ) {
    fprintf( stderr, "ERROR: Could not initialize capturing. \n" );
    getchar();
    exit(0);
  }

  std::cout<<"Sucessfully connected!"<<std::endl;

  /****************************************/
  //TEST 2: SEND MESSAGE TO THE SERVO SERVER
  /***************************************
  */

  float pos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  robot->control(JTRACK,pos, 8);
  while (true) {
    trackBall();
    getchar();
  }

  /****************************************/
  //TEST 3: TEAR DOWN
  /****************************************/
  delete robot;
  std::cout<<"Successfully disconnected!"<<std::endl;
}
