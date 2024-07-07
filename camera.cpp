// File to test the camera and C++ OpenCV libraries
// compile with
// g++ $(pkg-config --libs --cflags opencv) -o camera_test main.cpp
// run with
// ./cameratest

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "Camera.h"


using namespace cv;
using namespace std;
double distance_x;
double contour_area;
int contour_not_found = 0;
int collected = 0;
void * camera(void* )
{
  VideoCapture cap(0);
  if (!cap.isOpened()) {
    cerr << "ERROR: Unable to open the camera" << endl;
    return 0;
  }

   Mat frame;
  int coll = 0;

  cout << "Start grabbing, press a key on Live window to terminate" << endl;

  while(1) {
    cap >> frame;
    Rect crop_frames(0, 120, frame.cols, frame.rows - 120);
    frame = frame(crop_frames);
    if (frame.empty()) {
        cerr << "ERROR: Unable to grab from the camera" << endl;
        break;
    }
    Mat frame_hsv;
    cvtColor(frame,frame_hsv, COLOR_BGR2HSV);
    
    

    Mat mask1,mask, mask2;
    inRange(frame_hsv,Scalar(90,80,0),Scalar(110,255,255),mask);

    

    //medianBlur(mask1,mask,35); 
    Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));
    dilate(mask,mask,kernel,Point(-1,-1),2);
    erode(mask,mask,kernel,Point(-1,-1),2);
 


    vector<vector<Point> >contours;
    findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);


    if(!contours.empty())
    {
		contour_not_found = 0;
		Rect bounding_rect = boundingRect(contours[0]);
		contour_area = contourArea(contours[0]);
		int object_center_x = bounding_rect.x + bounding_rect.width /2;
		int object_center_y = bounding_rect.y + bounding_rect.height /2;

		int frame_center_x = frame.cols/2;
		int frame_center_y = frame.rows/2;
		
		distance_x = object_center_x - frame_center_x ;
		double distance_y = object_center_y - frame_center_y;
		//cout << "Distance: "<< distance_x << endl;
		//cout << "Contour area :" << contour_area << endl;
		//cout << " Contour width :" << bounding_rect.width << endl;
		//cout << " Contour height :" << bounding_rect.height << endl;
		int height = mask.rows;

	    // Calculate the y-coordinate of the region of interest (ROI) for cropping
		int y = height - 50;
	
	    // Create a rectangle for the ROI
	    Rect roi1(0, y, mask.cols, 50);
	    mask2  = mask(roi1);
	    //imshow("Live1",mask2);
	    int whitePixelCount = 0,count;
	    MatConstIterator_<uchar> it = mask2.begin<uchar>();
	    MatConstIterator_<uchar> end = mask2.end<uchar>();
	    for (; it != end; ++it)
		    {
		        if (*it == 255)  // Assuming white pixels have an intensity value of 255
		            ++whitePixelCount;
	                
	    	    }
	     //cout << "whitePixelCount: "<< whitePixelCount << endl;
	
	     if (whitePixelCount >12900 && contour_area > 14000)
	     {
			coll+=1;
	        if (coll >25)
	        {
				collected=1;
				coll =0;
				break;
	        }
		 }
		
		

	}
    else
    {
		contour_not_found = 1;
    }
    
    Mat frame_result;
    //bitwise_and(frame,frame,frame_result,mask);

    //imshow("Mask_hsv",mask);
    //imshow("Live1",frame);
    //imshow("Mask",mask1);
    //imshow("MAsk 2", mask2);
   // cout << "collected :" << collected <<endl;	
	
    int key = cv::waitKey(5);
    key = (key==255) ? -1 : key; //#Solve bug in 3.2.0
    if (key>=0)
      break;
  }
  cout << "Closing the camera" << endl;
  cap.release();
  destroyAllWindows();
  cout << "bye!" <<endl;
  
}



