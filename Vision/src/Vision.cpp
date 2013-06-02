#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <math.h>
//#include <cvblobs/BlobResult.h>

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

void circleTracking() {
	VideoCapture cap( CV_CAP_ANY );
	if ( !cap.isOpened() ) {
		fprintf( stderr, "ERROR: Could not initialize capturing. \n" );
		getchar();
	}
	// Create a window in which the captured images will be presented
	namedWindow( "video", CV_WINDOW_AUTOSIZE );

	// Show the image captured from the camera in the window and repeat
	while ( true ) {
		// Get one frame
		Mat frame, frame_gray, mask;
		cap >> frame;

//		// Convert to HSV
//		cvtColor(frame, mask, CV_BGR2HSV);
//		// Green Christmas Ornament
//		Scalar minHSV = Scalar(81, 100, 100);
//		Scalar maxHSV = Scalar(91, 255, 255);
//		inRange(mask, minHSV, maxHSV, frame_gray);
//		Mat element21 = getStructuringElement(MORPH_RECT, Size(21,21), Point(10,10));
//		morphologyEx(frame_gray, frame_gray, MORPH_OPEN, element21);
//		Mat element11 = getStructuringElement(MORPH_RECT, Size(11,11), Point(5,5));
//		morphologyEx(frame_gray, frame_gray, MORPH_CLOSE, element21);

		// Convert to grayscale
		cvtColor(frame, frame_gray, CV_BGR2GRAY);

		// Reduce noise
		GaussianBlur(frame_gray, frame_gray, Size(9,9), 2, 2);

		vector<Vec3f> circles;

		// Apply Hough Transform to find circles
		HoughCircles( frame_gray, circles, CV_HOUGH_GRADIENT, 1, frame_gray.rows/8, 200, 100, 0, 0 );
		// printf("There are %d circles!\n", circles.size());

		for (size_t i = 0; i < circles.size(); i++) {
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// circle outline
			circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
		}
		imshow( "video", frame );
//		imshow( "gray", frame_gray );


		// Wait for a keypress
		int c = cvWaitKey(100);
		if ( c != -1 ) break;
	}
}



IplImage* trackColor(IplImage *image, CvPoint *position, int hue) {
	CvScalar minHSV = cvScalar(hue, 100, 100);
	CvScalar maxHSV = cvScalar(hue+10, 255, 255);

	IplImage* imgHSV = cvCreateImage(cvGetSize(image), 8, 3);
	cvCvtColor(image, imgHSV, CV_BGR2HSV);
	IplImage* imgThreshed = cvCreateImage(cvGetSize(image), 8, 1);
	cvInRangeS(imgHSV,minHSV, maxHSV, imgThreshed);
	cvReleaseImage(&imgHSV);

	// Calculate the moments to estimate the position of the ball
	CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
	cvMoments(imgThreshed, moments, 1);
	double moment10 = cvGetSpatialMoment(moments, 1, 0);
	double moment01 = cvGetSpatialMoment(moments, 0, 1);
	double area = cvGetCentralMoment(moments, 0, 0);

	if (area < 100) {
		printf("Too noisy to track, detected area: %f\n", area);
		position->x = -1;
		position->y = -1;
	}
	else {
		position->x = moment10/area;
		position->y = moment01/area;
	}
	delete moments;
	return imgThreshed;
}

double calculateBearing(CvPoint red, CvPoint blue) {
	double a = blue.x-red.x;
	double b = blue.y-red.y;
	if (blue.y > red.y) {
		if (a == 0) {
			return 90.0;
		}
		else {
			return atan(b/a) * (180/3.14159);
		}
	}
	else {
		if (a == 0) {
			return 270.0;
		}
		else {
			return atan(b/a) * (180/3.14159) + 180;
		}
	}
}

void blobTracking() {
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	if ( !capture ) {
		fprintf( stderr, "ERROR: Could not initialize capturing. \n" );
		getchar();
	}
	// Create a window in which the captured images will be presented
	cvNamedWindow( "video", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "thresh", CV_WINDOW_AUTOSIZE);


	// Show the image captured from the camera in the window and repeat
	while ( true ) {
		// Get one frame
		IplImage* frame = cvQueryFrame( capture );
		if ( !frame ) {
		  fprintf( stderr, "ERROR: Could not retrieve frame.\n" );
		  getchar();
		  break;
		}


		CvPoint yellowPos;
		CvPoint bluePos;
		CvPoint redPos;
		IplImage* yellowThresh= trackColor(frame, &yellowPos, 25);
		IplImage* blueThresh= trackColor(frame, &bluePos, 100);
		IplImage* redThresh= trackColor(frame, &redPos, 164);

		// Draw a line only if positions are valid
		if (yellowPos.x > 0 && yellowPos.y > 0 && bluePos.x > 0 && bluePos.y > 0 && redPos.x && redPos.y) {
			cvLine(frame, yellowPos, bluePos, cvScalar(0, 255, 255), 5);
			cvLine(frame, yellowPos, redPos, cvScalar(0, 255, 255), 5);
			cvLine(frame, redPos, bluePos, cvScalar(0, 255, 255), 5);

			double bearing = calculateBearing(redPos, bluePos);
			printf("Bearing: %f\n",bearing);

		}

		cvShowImage("thresh", yellowThresh);
		cvShowImage("thresh", blueThresh);
		cvShowImage("thresh", redThresh);
		cvShowImage( "video", frame );
		cvReleaseImage(&yellowThresh);
		cvReleaseImage(&blueThresh);
		cvReleaseImage(&redThresh);

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
//	circleTracking();
//	blobTracking();


  return 0;
}
