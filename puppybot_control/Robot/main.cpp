
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
int counter = 0;
RobotCom* robot;
CvCapture* capture;


void RotateRobot(double DegCCRot);
IplImage* GetThresholdedImage(IplImage* img, CvScalar minHSV, CvScalar maxHSV);
bool findBallPos(int* lastX, int* lastY );
void CenterRobotOnBall(bool prec);
bool ballInWorkspace();

void RobotPositionReached() {
  float pos[8];
  while(true) {
    bool reached = true;
    robot->getStatus(GET_JPOS, pos);
    for (int i=0; i <2; i++){
      float diff = pos[i] - RobotPos[i];
      if (fabs(diff) >= 0.05) {
      //  printf("fail: %i %f\n", i, diff);
        reached = false;
      }
    }
    float diff = (pos[2]) - RobotPos[2];
    if (fabs(diff) >= 0.003) {
      reached = false;
    }
    for (int i=3; i <8; i++){
      float diff = pos[i] - RobotPos[i];
      if (i!=5  && fabs(diff) >= 0.05) {
      //  printf("fail: %i %f\n", i, diff);
        reached = false;
      }
    }

    if (reached) break;
    //printf("Position not reached %i\n", count);
    sleep(0.25);
  }
}

bool findBallPos(int& posX, int& posY) {
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

void RotateRobot(double degCCRot) {
  float radians = ((float)degCCRot)*3.14159 / 180.0;
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = radians;

  for(int i=0; i<8; i++) {
    RobotPos[i] = pos[i];
  }

  robot->control(JTRACK, pos, 8);
//  RobotPositionReached();
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
        printf ("Ball is center\n");
        break;
      }
    } else {
      // if ball not found, rotate 5 deg cc
      RotateRobot(3.0);
    }
  }
}

void RobotArmDown() {
  printf("Rotate robot arm\n");
  RobotPos[4] = 1.63;
  RobotPos[6] = 1.63;
  robot->control(JTRACK, RobotPos, 8);
  RobotPositionReached();
}

void MoveForward(double dist) {
  if (dist == 0) return;

  printf("Moving robot forward %f\n", dist);
  // RobotPos[0] = RobotPos[0] +(float) dist;//*cos(RobotPos[2]);
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  // pos[0] += ((float)dist)*cosf(pos[2]);
  // pos[1] += ((float)dist)*sinf(pos[2]);

  pos[0] = (float) dist;
  pos[1] = 0.0;
  pos[2] = 0.0;
  printf(" %f %f\n", pos[1], pos[2]);

  for(int i=0; i<1; i++) {
    RobotPos[i] = pos[i];
  }
  //pos[2] = RobotPos[2]; 

  //RobotPos[2] = 0.0;
  //RobotPos[1] = RobotPos[1] + dist*sin(RobotPos[2]);
  robot->control(JTRACK,pos, 8);
  // robot->control(JTRACK,RobotPos, 8);
//  RobotPositionReached();
}

void MoveForwardToBall() {
  int posX, posY;
  printf("move Forward To Ball\n");
  while (!ballInWorkspace()) {
    findBallPos(posX, posY);
    float dist = 30*0.0254;
    printf("Current posY: %i\n", posY);
    if (posY > 650) {
      float cube = posY*posY*posY;
      float sqr = posY*posY;
      dist = 0.00000024646*cube - 0.00021717*sqr + 0.073435*posY + 0.47217;
      dist = 0.0254*dist;
    } 
    dist = dist / 10.0;
    printf("Move Forward %f\n", dist);
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

void FloatAndReportJointAngles() {
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

void PickUpBall() {
  printf("Pick up ball\n");

  printf("open gripper\n");
  robot->controlGripper( (float)10 );
  
//  float RobotPos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float pos[8];
  robot->getStatus(GET_JPOS, pos);
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = 0.0;

  pos[4] = 0.5;
  //RobotPos[4] = 0.5;
  robot->control(JTRACK, pos, 8);
  sleep(1.5);
  pos[3] = 6*3.14159/180.0;
  pos[4] = 1.60;
  pos[6] = 1.3;
  robot->control(JTRACK, pos, 8);
  sleep(3);
  
  printf("Close Gripper\n");
  robot->controlGripper( (float)-5 );
  sleep(5);

  printf("raise arm\n");
  pos[7] = 1.57;
  pos[6] = 0.0;
  robot->control(JTRACK, pos, 8);
  sleep(3);

  for(int i=0; i<8; i++) {
//    RobotPos[i] = pos[i];
  }
}

void trackBall() {
    int frameWidth = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    int frameHeight = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
    int frameCenterX = frameWidth/2;
    int frameCenterY = frameHeight/2;

    int posX, posY;
    while(true) {
        printf("++++++++++++");
//        printf("findBallPos\n");
        bool onScreen = findBallPos(posX, posY);
        printf(" X: %i Y:%i\n", posX, posY);
        if (posX < 0 || posY < 0) {
            continue;
        }
        
        bool d1 = false;
        bool d2 = false;
        if (!onScreen) {
            printf("Ball not on screen, Rotate and search\n");
            MoveForward(0.0);
            RotateRobot(5.0);
        } else {
            if (posY > 1000) {
                printf("Ball far away\n");
                MoveForward(0.6);
            } else if (posY > 700) {
                printf("Ball close\n");
                MoveForward(0.05);
            } else if (posY < 680) {
                printf("Ball too close\n");
                MoveForward(-0.05);
            } else {
                printf("Ball right disatance\n");
                MoveForward(0.0);
                d1 = true;
            }
            
            printf("  %i > %i\n", posX, frameWidth);
            printf("  %i < %i\n", posX, frameCenterX);
            if (posX > (frameCenterX + 100)) {
                printf("Ball on left\n");
                RotateRobot(5.0);
            } else if (posX < (frameCenterX - 100)) {
                printf("Ball on right\n");
                RotateRobot(-5.0);
            } else {
                printf("Ball centered\n");
                RotateRobot(0.0);
                d2 = true;
            }
        }
        
        if (d1 && d2) {
            printf("Ball in workspace\n");
            break;
        }
        
    }
    printf("Pick up ball\n");
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
//  RotateRobot(5);
//  sleep(2);
//  RotateRobot(-5);
//  sleep(1);
//  MoveForward(0.01);
//  sleep(1);
//  MoveForward(0.0);
//  exit(0);
//   CenterRobotOnBall(false);
//   MoveForwardToBall();
    float pos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot->control(JTRACK,pos, 8);
    while (true) {
      trackBall();
      getchar();
    }
//  int posX, posY;
//  while(true) {
//    printf("findBallPos\n");
//    bool onScreen = findBallPos(posX, posY);
//    printf(" X: %i Y:%i\n", posX, posY);
//    if (posY > 500 && posY < 680) {
//      CenterRobotOnBall(true);
//      PickUpBall();
//      break;
//    }
//  }

  printf("DONE ARM MOVEMENT\n");
  while(true){}

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
