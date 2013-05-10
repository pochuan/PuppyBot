#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

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
	int hue = 20;
	int saturation = 100;
	int value = 100;
	  cvCreateTrackbar("Hue","thresh", &hue, 245, NULL);
	//  cvCreateTrackbar("Saturation","thresh", &saturation, 245, NULL);
	//  cvCreateTrackbar("Value","thresh", &value, 245, NULL);


	// Show the image captured from the camera in the window and repeat
	while ( true ) {
		// Get one frame
		IplImage* frame = cvQueryFrame( capture );
		if ( !frame ) {
		  fprintf( stderr, "ERROR: Could not retrieve frame.\n" );
		  getchar();
		  break;
		}

		if(imgScribble == NULL) {
			imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
		}

		CvScalar minHSV = cvScalar(81, 100, 100);
		CvScalar maxHSV = cvScalar(91, 255, 255);

//		    printf("Hue: %d Saturation: %d Value: %d \n", hue, saturation, value);
//		    CvScalar minHSV = cvScalar(hue, saturation, value);
//		    CvScalar maxHSV = cvScalar(hue+10, saturation+155, value+155);
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


		printf("Position (%d,%d)\n", posX, posY);

		// Draw a line only if positions are valid
		if (lastX > 0 && lastY > 0 && posX > 0 && posY > 0) {
		//    	printf("Draw line from (%d, %d) to (%d,%d)\n", posX, posY, lastX, lastY);
			cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0, 255, 255), 5);
		}

		// Combine the scribbling to the frame
		cvAdd(frame, imgScribble, frame);
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

int main( int argc, char** argv )
{
	colorTracking();


  return 0;
}
