//objectTrackingTutorial.cpp

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

//Note: Scalar(B,G,R)

#define _CRT_SECURE_NO_WARNINGS
#include <sstream>
#include <string>
#include <iostream>
#include <fstream> 
#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <math.h>
#include "rbt.h"
#include "lght.h"

using namespace cv;

/*****************************************************************************************
   Parameters in this section will be changed oftenly: 
 * 1.Capture window resolution
 * 2.Area threshold values.
 * Note: Due to light condition of surrounding area and the color you choose to filter 
 *       out, parameters below could varid a lot. So every time when operating envirnment
 *       changed, please also changed the parameter below accordingly.
 * TODO: Implement algorithm to perfom auto light adjustment.
 * */
//Size of capture window in width and height
const int FRAME_WIDTH = 1280;//640;
const int FRAME_HEIGHT = 720;//480;

//Light detection parameter: the pixel size of light source appear in camera.
const int MIN_LIGHT_DTECT_AREA = 250;
const int MAX_LIGHT_DTECT_AREA = 500;

/*Shape detection parameter: shape_area is used to identify robot body while head_area
  is used to identify robot head.
  Hints: 
  MAX_SHAPPE_DETECT_AREA could also be something like: FRAME_HEIGHT*FRAME_WIDTH/1.5 */
const int MIN_SHAPE_DETECT_AREA = 3000;
const int MAX_SHAPE_DETECT_AREA = 6000;
const int MIN_HEAD_AREA = 1200;
const int MAX_HEAD_AREA = 3000;
/**************************************************************************************/


/***************************************************************************************
  Parameter in this section won't change as often as last section
* */
//initial min and max HSV filter values.these will be changed using trackbars.
//once the color is fixed and the operating envirnoment is settled, you can 
//fix the value below.
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//initial min and max grayscale(light) filter values.
int G_MIN = 240;
int G_MAX = 256;

//constant and ratio
const double PI = 3.141592653589793;
//constant for mapping between camera pixel and real world unit (cm)
const double cm2pixleRatio = 0.095;
//constant for mapping between measured angle in openCV to angle sending to robot
//since robot only receives ASCII code: a 256 char table, and we only need 181 of 
//them to represent 0 to 180 degree. This is a ratio to map degree to its ASCII 
//code.
const double angleRatio = 1.9; 
//ASCII Code Constant
const int SPACE = 32;
const int ZERO  = 48;

//Max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 10;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "Thresholded Gray Image";
const string windowName2 = "Thresholded Hsv Image";
const string trackbarWindowName = "Trackbars";

/**************************************************************************************/

//return code for small object and shape detection functions
enum genral_RC {
	NOISY      = -1,
	NOTFOUND   =  0,
	//found robot 1
	FOUND_RBT1 =  1,
	//found robot 2
	FOUND_RBT2 =  2,
	//found two robots
	FOUND      =  3
};
//return code for robot rotation direction
enum rotation{
	CLKWS     = 49,
	CNTRCLKWS = 48
};

//This function gets called whenever a trackbar position is changed
void on_trackbar( int, void* )
{}

/** 
* this function convert int to string
* 
* Parameters:
* @param int number - an integer you want to change
*
* @return string - integer in string
*/
string intToString( int number )
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

/** 
* this function finds a cosine of angle between vectors
* from pt0->pt1 and from pt0->pt2.
* 
* Parameters:
* @param pt0 - 
* @param pt1 - 
* @param pt2 - 
*
* @return static double - cosine result for 0 to 180 degree
*/
static double cos_3pt( Point pt0, Point pt1, Point pt2 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/** 
* this function will create window with slide bars on it to control 
* color filtering and light source detection
*/
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
	
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, 256, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, 256, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, 256, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, 256, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, 256, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, 256, on_trackbar );
	createTrackbar( "G_MIN", trackbarWindowName, &G_MIN, G_MAX, on_trackbar );
    createTrackbar( "G_MAX", trackbarWindowName, &G_MAX, G_MAX, on_trackbar );
}

/** 
* use some of the openCV drawing functions to draw crosshairs on your tracked image!
* added 'if' and 'else' statements to prevent memory errors from writing off the 
* screen (ie. (-25,-25) is not within the window!)
* 
* Parameters:
* @param int x - x position of crosshairs center
* @param int y - y position of crosshairs center
* @param Mat& frame -  input and output of the frame that you want to 
* add crosshairs to.   
* @return void
*/
void drawAIM( int x, int y, Mat &frame)
{
	if (x != -1)
	{
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
}

/** 
* use some of the openCV drawing functions to draw crosshairs on your tracked image!
* added 'if' and 'else' statements to prevent memory errors from writing off the 
* screen (ie. (-25,-25) is not within the window!)
* 
* Parameters:
* @param Point2i pt - the center of crosshairs
* @param Mat& frame - input and output of the frame that you want to 
* add crosshairs to.   
* @return void
*/
void drawAIM( Point2i pt, Mat &frame)
{
	if (pt.x != -1)
	{
		circle(frame,pt,20,Scalar(0,255,0),2);
		if(pt.y-25>0)
		line(frame,pt,Point(pt.x,pt.y-25),Scalar(0,255,0),2);
		else line(frame,pt,Point(pt.x,0),Scalar(0,255,0),2);
		if(pt.y+25<FRAME_HEIGHT)
		line(frame,pt,Point(pt.x,pt.y+25),Scalar(0,255,0),2);
		else line(frame,pt,Point(pt.x,FRAME_HEIGHT),Scalar(0,255,0),2);
		if(pt.x-25>0)
		line(frame,pt,Point(pt.x-25,pt.y),Scalar(0,255,0),2);
		else line(frame,pt,Point(0,pt.y),Scalar(0,255,0),2);
		if(pt.x+25<FRAME_WIDTH)
		line(frame,pt,Point(pt.x+25,pt.y),Scalar(0,255,0),2);
		else line(frame,pt,Point(FRAME_WIDTH,pt.y),Scalar(0,255,0),2);
		putText(frame,intToString(pt.x)+","+intToString(pt.y),Point(pt.x,pt.y+30),1,1,Scalar(0,255,0),2);
	}
}

/** 
* this function will draw crosshairs on the center of robot body and robot head
* 
* Parameters:
* @param rbt robot - the robot object that you want to draw
* @param Mat& frame - input and output of the frame that you want to 
* add crosshairs to.   
* @return void
*/
void drawAIM( rbt robot, Mat &frame)
{
	if (robot.rbt_available())
	{
		drawAIM(robot.getHead(),frame);
		drawAIM(robot.getBody(),frame);
	}
}

/** 
* this function will put robot statistics on the upper left corner of output window
* 
* Parameters:
* @param Mat& frame - input and output of the frame that you want to 
* @param rotation rt - the direction of rotation of robot
* @param double ang - angle of robot to rotate
* @param double dis - distance of robot to move
* @param int functionCode - whether update statistic in file or statistic in real world unit
*  
* @return void
*/
void drawData(Mat& cameraFeed, rotation rt, double ang, double dis, int functionCode)
{
	if(functionCode == 1)
	{
		string txtData = "data r:" + intToString((int)rt) + ",a:" + intToString((int)ang) + ",d:" + intToString((int)dis);
		putText(cameraFeed,txtData,Point(0,80),1,1,Scalar(0,255,0),2);
	}
	else
	{
		string txtData = "real r:" + intToString((int)rt) + ",a:" + intToString((int)ang) + ",d:" + intToString((int)dis);
		putText(cameraFeed,txtData,Point(0,110),1,1,Scalar(0,255,0),2);
	}
}

/** 
* this function saves distance and robot between robot and light to a .txt
* this function decides which robot will be closer to the light and save the location 
* of that robot into pt_workRBT.
* prerequisite:
* findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE ) is called already.
* 
* Parameters:
* @param pt_Robot - 
* @param contours - 
* @return void
* 
* rotation direction 1: clockwise. 0: counter clock
*/
void angleDistanceSave( Mat& cameraFeed, rbt robot1, rbt robot2, lght light )
//TODO: add support for two robots, two lights
{
	int dis = 0;
	rotation rotation1 = CLKWS; 
	rotation rotation2 = CLKWS;
	double angle1 = 0;
	double angle2 = 0;
	double distance1 = 0;
	double distance2 = 0;
	double min_dis1 = 9999999;
	double min_dis2 = 9999999;
	unsigned int index1_min;
	unsigned int index2_min;

	if (robot1.rbt_available())
	{
		for(unsigned int i=0; i<light.getIndex(); i++)
		{
			distance1 = sqrt(pow( (robot1.getBody().x - light.getLgt(i).x), 2 ) + pow( (robot1.getBody().y - light.getLgt(i).y), 2 ));
			//if the new distance is smaller than minimum distance
			if(distance1 < min_dis1)
			{
				min_dis1 = distance1;
				index1_min = i;
			}
		}
	}
	if (robot2.rbt_available())
	{
		for(unsigned int i=0; i<light.getIndex(); i++)
		{
			distance2 = sqrt(pow( (robot2.getBody().x - light.getLgt(i).x), 2 ) + pow( (robot2.getBody().y - light.getLgt(i).y), 2 ));
			//if the new distance is smaller than minimum distance
			if(distance2 < min_dis2)
			{
				min_dis2 = distance1;
				index2_min = i;
			}
		}	
	}


	if (robot1.rbt_available() && robot2.rbt_available() && light.lght_available())
	{
		if (min_dis1 < min_dis2)
		{
			angle1 = (acos(cos_3pt(robot1.getBody(),robot1.getHead(),light.getLgt(index1_min)))/PI)*180;
			dis = (int)distance1;

			Point2i vBL = light.getLgt(index1_min) - robot1.getBody();
			Point2i vBH = robot1.getHead() - robot1.getBody();
			int crossProduct = vBL.x * vBH.y - vBL.y * vBH.x;
			if(crossProduct > 0)
			{
				rotation1 = CNTRCLKWS;
			}
 		}
		else 
		{
			angle2 = (acos(cos_3pt(robot2.getBody(),robot2.getHead(),light.getLgt(index2_min)))/PI)*180;
			dis = (int)distance2;

			Point2i vBL = light.getLgt(index2_min) - robot2.getBody();
			Point2i vBH = robot2.getHead() - robot2.getBody();
			int crossProduct = vBL.x * vBH.y - vBL.y * vBH.x;
			if(crossProduct > 0)
			{
				rotation2 = CNTRCLKWS;
			}
		}
	}
	else if(robot1.rbt_available() && !robot2.rbt_available() && light.lght_available())
	{
		angle1 = (acos(cos_3pt(robot1.getBody(),robot1.getHead(),light.getLgt(index1_min)))/PI)*180;
		dis = (int)distance1;

		Point2i vBL = light.getLgt(index1_min) - robot1.getBody();
		Point2i vBH = robot1.getHead() - robot1.getBody();
		int crossProduct = vBL.x * vBH.y - vBL.y * vBH.x;
		if(crossProduct > 0)
		{
			rotation1 = CNTRCLKWS;
		}
	}
	else if(!robot1.rbt_available() && robot2.rbt_available() && light.lght_available())
	{
		angle2 = (acos(cos_3pt(robot2.getBody(),robot2.getHead(),light.getLgt(index2_min)))/PI)*180;
		dis = (int)distance2;

		Point2i vBL = light.getLgt(index2_min) - robot2.getBody();
		Point2i vBH = robot2.getHead() - robot2.getBody();
		int crossProduct = vBL.x * vBH.y - vBL.y * vBH.x;
		if(crossProduct > 0)
		{
			rotation2 = CNTRCLKWS;
		}
	}


	std::ofstream myfile1;
	myfile1.open ("D:/OpenCV/data1.txt");
	angle1 = (int)(angle1/angleRatio + 31);
	distance1 = (int)(distance1 * cm2pixleRatio / 1.5 + 31);
	
	myfile1 << (char)rotation1 << (char)SPACE << (char)angle1 << (char)SPACE << (char)distance1;
	myfile1.close();
	drawData(cameraFeed,rotation1,angle1,distance1,1);
	drawData(cameraFeed,rotation1,(angle1-31)*angleRatio,(distance1 - 31)*1.5,2);
	//std::cout << "data1:" << rotation1 << " " << angle1 << " " << distance1 << std::endl;
	//std::cout << "\n" << "ang:" << (int)((angle1-31)*angleRatio) << ", dis:" << (int)((distance1 - 31)*1.5) << std::endl;


	//std::ofstream myfile2;
	//myfile2.open ("D:/OpenCV/data2.txt");
	//
	//angle2 = (int)(angle2/angleRatio + 31);
	//distance2 = (int)(distance2*pixle2cmRatio/1.5 +31);
	//
	//myfile2 << (char)rotation2 << (char)SPACE << (char)angle2 << (char)SPACE << (char)distance2;
	//myfile2.close();
	//std::cout << "rbt2:" << rotation2 << " " << angle2 << " " << distance2*pixle2cmRatio << std::endl;
}

void drawFrameHSV(Mat& HSV, genral_RC RC_trackObj, genral_RC RC_trackLgt, rbt robot1, lght light)
{
	bool rbt = false;

	if (RC_trackObj == FOUND_RBT1)
	{
		putText(HSV,"Found Robot1",Point(0,20),2,1,Scalar(255,0,0),2);
		//draw object location on screen
		drawAIM(robot1,HSV);
		rbt = true;
	}
	else if (RC_trackObj == NOISY)
	{
		putText(HSV,"Rbt:NOISY!",Point(0,20),2,1,Scalar(255,0,0),2);
	}
	else putText(HSV,"Searching...",Point(0,20),1,1,Scalar(255,0,0),2);


	if (RC_trackLgt == FOUND)
	{
		for(unsigned int i=0; i < light.getIndex(); i++)
		{
			putText(HSV,"Found Light",Point(0,50),2,1,Scalar(255,0,0),2);
			drawAIM(light.getLgt(i),HSV);
		}
	}
	else if (RC_trackLgt == NOISY)
	{
		putText(HSV,"Lgt:NOISY!",Point(0,50),2,1,Scalar(255,0,0),2);
	}
	else putText(HSV,"Searching...",Point(0,50),1,1,Scalar(255,0,0),2);

}

void drawFrame(Mat& cameraFeed, genral_RC RC_trackObj, genral_RC RC_trackLgt, rbt robot1, rbt robot2, lght light)
//TODO: To make this function simpler
{
	bool rbt = false;

	if(RC_trackObj == FOUND)
	{
		putText(cameraFeed,"BAZINGA!",Point(0,20),2,1,Scalar(0,255,0),2);
		//draw object location on screen
		drawAIM(robot1,cameraFeed);
		drawAIM(robot2,cameraFeed);
		rbt = true;
	}
	else if (RC_trackObj == FOUND_RBT1)
	{
		putText(cameraFeed,"Found Robot1",Point(0,20),2,1,Scalar(0,235,0),2);
		//draw object location on screen
		drawAIM(robot1,cameraFeed);
		rbt = true;
	}
	else if (RC_trackObj == FOUND_RBT2)
	{
		putText(cameraFeed,"Found Robot2",Point(0,20),2,1,Scalar(0,235,0),2);
		//draw object location on screen
		drawAIM(robot2,cameraFeed);
		rbt = true;
	}
	else if (RC_trackObj == NOISY)
	{
		putText(cameraFeed,"Rbt:NOISY!",Point(0,20),2,1,Scalar(0,0,255),2);
	}
	else putText(cameraFeed,"Searching...",Point(0,20),1,1,Scalar(255,0,0),2);


	if (RC_trackLgt == FOUND)
	{
		for(unsigned int i=0; i < light.getIndex(); i++)
		{
			putText(cameraFeed,"Found Light",Point(0,50),2,1,Scalar(0,255,0),2);
			drawAIM(light.getLgt(i),cameraFeed);
		}
	}
	else if (RC_trackLgt == NOISY)
	{
		putText(cameraFeed,"Lgt:NOISY!",Point(0,50),2,1,Scalar(0,0,255),2);
	}
	else putText(cameraFeed,"Searching...",Point(0,50),1,1,Scalar(255,0,0),2);

	//if (rbt && RC_trackLgt == FOUND)
	angleDistanceSave(cameraFeed,robot1,robot2,light);
}

void morphOps( Mat &thresh )
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

/** 
* this function can detect small object, such as robot head
* prerequisite:
* findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE ) is called already.
* 
* Parameters:
* @param pt_Robot - 
* @param contours - 
*
* @return void
*/
genral_RC shapeDetect( rbt* p_Robot1, rbt* p_Robot2, vector< vector<Point> > contours )
{
	genral_RC RC = NOTFOUND;
	genral_RC rbt1Cond = NOTFOUND;
	genral_RC rbt2Cond = NOTFOUND;
	vector<Point> approx;
	if (contours.size() > 0)
	{
		//contours.size(): number of objects;
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (contours.size() < MAX_NUM_OBJECTS)
		{
				for (unsigned int i = 0; i < contours.size(); i++) 
				{	
					approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.06, true); //FIXME
					Moments moment = moments((cv::Mat)contours[i]);
					double area = moment.m00;
		
					//detect for square/robot	
					if( approx.size() == 4 && area > MIN_SHAPE_DETECT_AREA && area < MAX_SHAPE_DETECT_AREA && isContourConvex(Mat(approx)) ) //FIXME
					{
						double maxCosine = 0;
						for( int j = 2; j < 5; j++ )
						{
							// find the maximum cosine of the angle between joint edges
							double cosine = fabs(cos_3pt(approx[j-1], approx[j%4], approx[j-2]));
							maxCosine = MAX(maxCosine, cosine);
						}
						if( maxCosine >= 0 && maxCosine < 0.3 )
						{
							rbt1Cond = FOUND_RBT1;
							p_Robot1->setHead( (int)(moment.m10/area), (int)(moment.m01/area) );
						}
					}
		
					//detect for circle/light	
					else if( approx.size() == 8 && area > MIN_SHAPE_DETECT_AREA && area < MAX_SHAPE_DETECT_AREA && isContourConvex(Mat(approx)) ) //FIXME
					{ 
						rbt2Cond = FOUND_RBT2;
						p_Robot2->setHead( (int)(moment.m10/area), (int)(moment.m01/area) );
					}

					////detection for pentagon
					//else if( approx.size() == 5 && area > MIN_SHAPE_DETECT_AREA && area < MAX_SHAPE_DETECT_AREA && isContourConvex(Mat(approx)) )
					//{
					//	double maxCosine = 0;
					//	int correctAngleCount = 0;
				 //       for( int j = 0; j < 5; j++ )
				 //       {
				 //           // find the maximum cosine of the angle between joint edges
				 //           double cosine = fabs(cos_3pt(approx[j], approx[(j+2)%5], approx[(j+1)%5]));
					//		if( cosine >= -0.4226 && cosine <= 0 ) //-0.4226:115degree;0:90degree;
					//		{
					//			correctAngleCount++;
					//		}
					//	}
					//	if( correctAngleCount == 5 )
					//	{
					//		int x = moment.m10/area;
					//		int y = moment.m01/area;
					//	}
					//}
				}
				
				if( rbt1Cond+rbt2Cond == FOUND )
				{
					RC = FOUND;
				}
				else if ( rbt1Cond+rbt2Cond == FOUND_RBT1 )
				{
					RC = FOUND_RBT1;
				}
				else if ( rbt1Cond+rbt2Cond == FOUND_RBT2 )
				{
					RC = FOUND_RBT2;
				}

		}
		else RC = NOISY;
	}
	
	return RC;
}

/** 
* this function is used as a help function for headDetect
*
* prerequisite:
* findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE)
* headDetect(rbt* p_Robot1, rbt* p_Robot2, vector< vector<Point> > contours, vector<Vec4i> hierarchy)
* Parameters:
* @param p_Robot - 
* @param contours - 
*
* @return genral_RC
*/
genral_RC shapeDetect( rbt* p_Robot1, rbt* p_Robot2, vector<Point> contours )
{
	genral_RC RC = NOTFOUND;
	genral_RC rbt1Cond = NOTFOUND;
	genral_RC rbt2Cond = NOTFOUND;
	vector<Point> approx;
	if (contours.size() > 0)
	{

		approxPolyDP(Mat(contours), approx, arcLength(Mat(contours), true)*0.1, true);
		Moments moment = moments((cv::Mat)contours);
		double area = moment.m00;
		
		//detect for square	
		if( approx.size() == 4 && isContourConvex(Mat(approx)) )
		{
			double maxCosine = 0;
			for( int j = 2; j < 5; j++ )
			{
				//find the maximum cosine of the angle between joint edges
				double cosine = fabs(cos_3pt(approx[j-1], approx[j%4], approx[j-2]));
				maxCosine = MAX(maxCosine, cosine);
			}
			if( maxCosine >= 0 && maxCosine < 0.3 )
			{
				rbt1Cond = FOUND_RBT1;
				p_Robot1->setBody( (int)(moment.m10/area), (int)(moment.m01/area) );
			}
		}
		
		//detect for circle	
		else if( approx.size() == 8 && isContourConvex(Mat(approx)) )
		{ 
			rbt2Cond = FOUND_RBT2;
			p_Robot2->setBody( (int)(moment.m10/area), (int)(moment.m01/area) );
		}
				
		if( rbt1Cond+rbt2Cond == FOUND )
		{
			RC = FOUND;
		}
		else if ( rbt1Cond+rbt2Cond == FOUND_RBT1 )
		{
			RC = FOUND_RBT1;
		}
		else if ( rbt1Cond+rbt2Cond == FOUND_RBT2 )
		{
			RC = FOUND_RBT2;
		}

	}
	
	return RC;
}

/** 
* this function can detect small object, such as robot head
* prerequisite:
* findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE ) is called already.
* 
* Parameters:
* @param smallObj - 
* @param contours - 
* @param hierarchy - 
*
* @return int
*/
genral_RC headDetect( rbt* p_Robot1, rbt* p_Robot2, vector< vector<Point> > contours, vector<Vec4i> hierarchy )
{
	genral_RC RC = NOTFOUND;

	if (hierarchy.size() > 0) 
	{
		//hierarchy.size(): number of objects;
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if (hierarchy.size() < 20)
		{
			//hierarchy[index][0] = next object
			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
			{ 
				//use moments method to find our filtered object
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

                if (area > MIN_HEAD_AREA && area < MAX_HEAD_AREA) //FIXME
				{
					RC = shapeDetect(p_Robot1,p_Robot2,contours[index]);
				}
			}
		}
		else RC = NOISY;
	}
	
	return RC;
}

genral_RC trackObject( Mat threshold, Mat &cameraFeed, rbt* p_rbt1, rbt* p_rbt2)
{
	Mat temp;
	genral_RC headFound  = NOTFOUND;
	genral_RC shapeFound = NOTFOUND;

	//Copies the matrix to a temp which will be modified later.
	//We don't want to modify the original matrix threshold.
	threshold.copyTo(temp); 
	//these two vectors needed for output of findContours
	//contours: every contour location in a group of points 
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
	
	//find the robots by shape detection and area difference
	shapeFound = shapeDetect(p_rbt1, p_rbt2, contours);
	headFound  = headDetect(p_rbt1,p_rbt2,contours,hierarchy);

	if(shapeFound == FOUND_RBT1 && headFound == FOUND_RBT1)  return FOUND_RBT1;
	else if(shapeFound == FOUND_RBT2 && headFound == FOUND_RBT2)  return FOUND_RBT2;
	else if(shapeFound == FOUND && headFound == FOUND)  return FOUND;
	else if (headFound == NOISY) return NOISY;
	else return NOTFOUND;
}

genral_RC trackLight( Mat threshold, Mat &cameraFeed, lght* light)
//TODO: add functionality to support more than one lights
{
	Mat temp;
	genral_RC RC = NOTFOUND;

	//Copies the matrix to another one.
	threshold.copyTo(temp); 
	//these two vectors needed for output of findContours
	//contours: every contour location in a group of points 
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
	if (hierarchy.size() > 0)
	{
		if (hierarchy.size() <= 3)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				if (area > MIN_LIGHT_DTECT_AREA && area < MAX_LIGHT_DTECT_AREA) //FIXME
				{
					light->addLgt((int)(moment.m10/area),(int)(moment.m01/area));
					RC = FOUND;
				}
			}
		}
		else RC = NOISY;
	}
	return RC;
}

int main(int argc, char* argv[])
{
	//some boolean variables for different functionality within this
	//program
    bool useMorphOps  = true;
	bool trackObjects = true;
	bool trackLights  = true;

	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat HSV,GRY;
	//matrix storage for binary threshold image
	Mat threshold_HSV,threshold_GRY;
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
		genral_RC RC_trackObj = NOTFOUND;
		genral_RC RC_trackLgt = NOTFOUND;
		rbt robot1,robot2;
		lght light;

		//store image to matrix
		capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace and grayscale space
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		cvtColor(cameraFeed,GRY,CV_BGR2GRAY);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold_HSV);
		inRange(GRY, G_MIN, G_MAX, threshold_GRY);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps)
			morphOps(threshold_HSV);
			morphOps(threshold_GRY);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if(trackObjects)
			 RC_trackObj = trackObject(threshold_HSV,cameraFeed, &robot1, &robot2);
		if(trackLights)
			 RC_trackLgt = trackLight(threshold_GRY, cameraFeed, &light);
		
		drawFrame(cameraFeed, RC_trackObj, RC_trackLgt, robot1, robot2, light);
		drawFrameHSV(threshold_HSV, RC_trackObj, RC_trackLgt, robot1, light);

		//show frames 
		imshow(windowName1,threshold_GRY);
		imshow(windowName2,threshold_HSV);
		imshow(windowName,cameraFeed);

		//delay 30ms so that screen can refresh.
		waitKey(30);
	}

	return 0;
}
