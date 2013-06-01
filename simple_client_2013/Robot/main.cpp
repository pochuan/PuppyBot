
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

using namespace cv;


IplImage* GetThresholdedImage(IplImage* img, CvScalar minHSV, CvScalar maxHSV) {
	// Convert img to HSV space
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
	cvInRangeS(imgHSV,minHSV, maxHSV, imgThreshed);
	cvReleaseImage(&imgHSV);
	return imgThreshed;
}

void colorTracking() {
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	if ( !capture ) {
		fprintf( stderr, "ERROR: Could not initialize capturing. \n" );
		getchar();
	}
	// Create a window in which the captured images will be presented
	cvNamedWindow( "video", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "thresh", CV_WINDOW_AUTOSIZE);
	IplImage* imgScribble = NULL;
	int posX = 0;
	int posY = 0;
	int hue = 90;
	int saturation = 150;
	int value = 150;
	  cvCreateTrackbar("Hue","thresh", &hue, 245, NULL);
	  cvCreateTrackbar("Saturation","thresh", &saturation, 245, NULL);
	  cvCreateTrackbar("Value","thresh", &value, 245, NULL);


	// Show the image captured from the camera in the window and repeat
	while ( true ) {
		// Get one frame
		IplImage* frame = cvQueryFrame( capture );
		if ( !frame ) {
		  fprintf( stderr, "ERROR: Could not retrieve frame.\n" );
		  getchar();
		  break;
		}

//		if(imgScribble == NULL) {
//			imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
//		}

		// Green Christmas Ornament
//		CvScalar minHSV = cvScalar(30, 150, 150);
//		CvScalar maxHSV = cvScalar(65, 255, 255);
//		CvScalar minHSV = cvScalar(hue, saturation, value);
//		CvScalar maxHSV = cvScalar(hue+10, saturation+200, value+200);
		CvScalar minHSV = cvScalar(30, saturation, value);
		CvScalar maxHSV = cvScalar(65, 255, 255);

	    printf("Hue: %d Saturation: %d Value: %d \n", hue, saturation, value);
		IplImage* imgThresh = GetThresholdedImage(frame, minHSV, maxHSV);

		// Calculate the moments to estimate the position of the ball
		CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
		cvMoments(imgThresh, moments, 1);
		double moment10 = cvGetSpatialMoment(moments, 1, 0);
		double moment01 = cvGetSpatialMoment(moments, 0, 1);
		double area = cvGetCentralMoment(moments, 0, 0);


		int lastX = posX;
		int lastY = posY;
		posX = moment10/area;
		posY = moment01/area;
//		printf("Area: %f\n", area);

		printf("Position (%d,%d)\n", posX, posY);

		// Draw a line only if positions are valid
		if (area > 100) {
			if (lastX > 0 && lastY > 0 && posX > 0 && posY > 0) {
			//    	printf("Draw line from (%d, %d) to (%d,%d)\n", posX, posY, lastX, lastY);
//				cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0, 255, 255), 5);
				cvLine(frame, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0, 255, 255), 5);
			}
		}
		else printf("Too noisy to track\n");

		// Combine the scribbling to the frame
//		cvAdd(frame, imgScribble, frame);
		//    cvShowImage("scribble", imgScribble);

		cvShowImage("thresh", imgThresh);
		cvShowImage( "video", frame );
		cvReleaseImage(&imgThresh);
		delete moments;

		// Wait for a keypress
		int c = cvWaitKey(10);
		if ( c != -1 ) break;
	}
	// Release the capture device housekeeping
	cvReleaseCapture( &capture );
	cvDestroyWindow( "mywindow" );

}

int main(int argc, char** argv)
{
        colorTracking();
	std::cout<<"This program tests the network connectivity between the client simulator and the servo server"<<std::endl;

/****************************************/
//TEST 1: ESTABLISH CONNECTION
/****************************************/
	RobotCom *test_robot = new RobotCom();
	
	std::cout<<"Sucessfully connected!"<<std::endl;

/****************************************/
//TEST 2: SEND MESSAGE TO THE SERVO SERVER
/****************************************/
	
	float helloWorld[] = {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	test_robot->control( JTRACK, helloWorld, 8);
	

/****************************************/
//TEST 3: RECEIVE REPLIES FROM THE SERVO SERVER
/****************************************/
	float data_in[8];
	test_robot->getStatus(GET_JPOS, data_in);
	std::cout<<"Joint angles are "<<data_in[0]<<" "<<data_in[1]<<" "<<data_in[2]<<" "<<data_in[3]<<" "<<data_in[4]<<" "<<data_in[5]<<" "<<data_in[6]<<" "<<data_in[7]<<std::endl;

/****************************************/
//TEST 4: TEAR DOWN
/****************************************/

	delete test_robot;

	std::cout<<"Successfully disconnected!"<<std::endl;	

}
