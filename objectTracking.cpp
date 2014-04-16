//objectTrackingTutorial.cpp

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

#define _CRT_SECURE_NO_WARNINGS
#include <sstream>
#include <string>
#include <iostream>
#include <fstream> //for saveXY
#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <math.h>

using namespace cv;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int PHOTO_ON = 1;
int PHOTO_OFF = 0;
//default capture width and height
const int FRAME_WIDTH = 1280;//640;
const int FRAME_HEIGHT = 720;//480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_SHAPE_DETECT_AREA = 2000;//100*100;
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
//const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
//const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

void on_trackbar( int, void* )
{//This function gets called whenever a trackbar position is changed
}

string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void createTrackbars()
{
	//create window for trackbars
    namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//sprintf( TrackbarName, "Photo", PHOTO_OFF);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
	createTrackbar( "Photo", trackbarWindowName, &PHOTO_OFF, PHOTO_ON, on_trackbar );
}

//void saveXY(int x_rbt, int y_rbt, int x_src, int y_src )
////this function can save 
//{
//	ofstream myfile;
//	myfile.open ("example.txt");
//	myfile << "Writing this to a file.\n";
//	myfile.close();
//}

void drawObject(int x, int y, Mat &frame)
{
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
}

void morphOps(Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	
}

void shapeDetect(Mat &cameraFeed, vector< vector<Point> > contours)
{
		vector<Point> approx;
		int x_sqr_last = -1;
		int y_sqr_last = -1;
		int x_circle_last = -1;
		int y_circle_last = -1;
		int x_sqr_curnt;
		int y_sqr_curnt;
		int x_circle_curnt;
		int y_circle_curnt;

		for (int i = 0; i < contours.size(); i++) 
		{	
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
			Moments moment = moments((cv::Mat)contours[i]);
			double area = moment.m00;
			//detection for square
			if( approx.size() == 4 && area > MIN_SHAPE_DETECT_AREA && area < MAX_OBJECT_AREA && isContourConvex(Mat(approx)) )
			{
				double maxCosine = 0;
                for( int j = 2; j < 5; j++ )
                {
                    // find the maximum cosine of the angle between joint edges
                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    maxCosine = MAX(maxCosine, cosine);
				}
				if( maxCosine >= 0 && maxCosine < 0.3 )
				{
					x_sqr0 = moment.m10/area;
					y_sqr0 = moment.m01/area;
					const Point* p = &approx[0];
					int n = (int)approx.size();
				    polylines(cameraFeed, &p, &n, 1, true, Scalar(0,0,255), 100, CV_AA); //Scalar(B,G,R)
			    }
			}
			//detection for pentagon
			//else if( approx.size() == 5 && area > MIN_SHAPE_DETECT_AREA && area < MAX_OBJECT_AREA && isContourConvex(Mat(approx)) )
			//{
			//	double maxCosine = 0;
			//	int correctAngleCount = 0;
   //             for( int j = 0; j < 5; j++ )
   //             {
   //                 // find the maximum cosine of the angle between joint edges
   //                 double cosine = fabs(angle(approx[j], approx[(j+2)%5], approx[(j+1)%5]));
			//		if( cosine >= -0.4226 && cosine <= 0 ) //-0.4226:115degree;0:90degree;
			//		{
			//			correctAngleCount++;
			//		}
			//	}
			//	if( correctAngleCount == 5 )
			//	{
			//		x = moment.m10/area;
			//		y = moment.m01/area;
			//		const Point* p = &approx[0];
			//		int n = (int)approx.size();
			//	    polylines(cameraFeed, &p, &n, 1, true, Scalar(0,255,128), 100, CV_AA); //Scalar(B,G,R)
			//	}
			//}
			//detection for circle
			else if( approx.size() == 8 && area > MIN_SHAPE_DETECT_AREA && area < MAX_OBJECT_AREA && isContourConvex(Mat(approx)) )
			{
					x_circle = moment.m10/area;
					y_circle = moment.m01/area;
					const Point* p = &approx[0];
					int n = (int)approx.size();
				    polylines(cameraFeed, &p, &n, 1, true, Scalar(0,255,128), 100, CV_AA); //Scalar(B,G,R)
			}
		}

}

void trackFilteredObject(Mat threshold, Mat &cameraFeed)
{
	Mat temp;
	double refArea = 0; // a reference area used for object detect.
	bool objectFound = false;
	int x0,x1,y0,y1;

	threshold.copyTo(temp); //Copies the matrix to another one.
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;//contours 有所有contour的位置
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	shapeDetect(cameraFeed,contours);

	if (hierarchy.size() > 0) 
	{
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0]) //hierarchy[index][0] = next object
			{ 
				//use moments method to find our filtered object
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
				{
					x0 = moment.m10/area;
					y0 = moment.m01/area;
					if (objectFound == false)
					{
						x1 = x0;
						y1 = y0;
					}
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
			{
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x0,y0,cameraFeed);
				drawObject(x1,y1,cameraFeed);
			}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

int main(int argc, char* argv[])
{
	//some boolean variables for different functionality within this
	//program
    bool trackObjects = true;
    bool useMorphOps = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	//create slider bars for HSV filtering
	createTrackbars();
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1)
	{
		//store image to matrix
		capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold); //threshold是hsv在mx和mn范围内的binary mtx
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps)
			morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if(trackObjects)
			trackFilteredObject(threshold,cameraFeed);

		//show frames 
		imshow(windowName2,threshold);
		imshow(windowName,cameraFeed);
		if (PHOTO_OFF == 1)
		{
			imwrite("D:/OpenCV/openCV_studio/clrfiltr/ImgCptr2.jpg",cameraFeed);
			PHOTO_OFF = 0;
		}
		//imshow(windowName1,HSV);

		//delay 30ms so that screen can refresh.
		waitKey(30);
	}

	return 0;
}

