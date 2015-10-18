#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdbool.h>
#include <ctime>
#include <signal.h>
#include <sys/time.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <fstream>
// #include "detectionAlgo.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<fstream>
#include <iostream>
#include <ctype.h>
#include<iostream>


#include <lcm/lcm-cpp.hpp>
#include "../lcmtypes/vision_state_t.hpp"

lcm::LCM lcm1;
vision_state_t vision_msg;

using namespace std;
using namespace cv;



int CAM_VMIN = 10,CAM_VMAX = 256, CAM_SMIN = 35, selectObject = false,trackObject = 0;
bool backprojMode = false;
bool showHist = true;
Point origin;
Rect selection;

int FRAME_W = 1920;
int FRAME_H = 1080;
int frame_number;
Mat image,frame;


//Calculation of real pixel numbers from simple perspective projection assumption and flat quadrotor assumption
float hgtEst(int pix,float pp, float fl, float rs)
{
        float hgt;
        if (!pp || !fl || !rs )
        {
                cout<<"Parameter value in real pixel number calculation is 0. Please check"<<endl;
                exit(1);
        }

        //Formula real size = pixel pitch * pixels * real distance / focal length
        hgt = rs*fl/pp/pix ;
        return hgt;
}


float persProj(float x, float Z, float f)
{
        float X;
        if (!x || !Z || !f)
        {
                cout<<"Parameter value in perspective projection is 0. Please check"<<endl;
                exit(1);
        }

        //Formula is x = X/Z*f (image x = real X / real height Z * focal length)
        X = x/f*Z;
        return X;
}




static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
        {
	    destroyAllWindows();	
            trackObject = -1;
        }
        break;
    }
}

int main()

{

  if(!lcm1.good()) return 1;

  VideoCapture cap(1);
	cap >> frame;
		
	float x=0,y=0,z=0;
	int estPixel;


	 //Temporary local variables
        Rect trackWindow;
	int hsize = 16;
        float hranges[] = {0,180};
        const float* phranges = hranges;
        Mat hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
        bool paused = false;
	int _vmin,_vmax;
	namedWindow( "CamShift Demo", 0 );
	setMouseCallback( "CamShift Demo", onMouse, 0 );

	while(!frame.empty())
	{
		
		RotatedRect trackBox;
                if( !paused )
                {

	                 //resize(frame,frame,Size(FRAME_W,FRAME_H));    
                         frame_number = cap.get(CV_CAP_PROP_POS_FRAMES);
                 }

                 frame.copyTo(image);
                 if( !paused )
                 {
	                  cvtColor(image, hsv, COLOR_BGR2HSV);
                          if( trackObject )
                          {
	                           _vmin = CAM_VMIN, _vmax = CAM_VMAX;
			           inRange(hsv, Scalar(0, CAM_SMIN, MIN(_vmin,_vmax)),
                                   Scalar(180, 256, MAX(_vmin, _vmax)), mask);

                                   int ch[] = {0, 0};

                                   hue.create(hsv.size(), hsv.depth());
                                   mixChannels(&hsv, 1, &hue, 1, ch, 1);

                                   if( trackObject < 0 )
                                   {
					destroyAllWindows();
                                   	Mat roi(hue, selection), maskroi(mask, selection);
                                        calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                                        normalize(hist, hist, 0, 255, CV_MINMAX);
                                        trackWindow = selection;
                                        trackObject = 1;
	                                histimg = Scalar::all(0);
          
	                                int binW = histimg.cols / hsize;
                                        Mat buf(1, hsize, CV_8UC3);

                                        for( int i = 0; i < hsize; i++ )

						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);

                                        cvtColor(buf, buf, CV_HSV2BGR);

                                        for( int i = 0; i < hsize; i++ )
                                        {

						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);

						rectangle( histimg, Point(i*binW,histimg.rows),Point((i+1)*binW,histimg.rows - val),Scalar(buf.at<Vec3b>(i)), -1, 8 );

                                        }
                                 }

                                 calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                                 backproj &= mask;
                                 trackBox = CamShift(backproj, trackWindow,
                                 TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

                                 if( trackWindow.area() <= 1 )
                                 {
  	                               int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
        	                       trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,trackWindow.x + r, trackWindow.y + r) &Rect(0, 0, cols, rows);

                                       if (trackWindow.area() == 0)

						trackWindow = Rect(360,240,100,100);
                                 }
			rectangle(image,trackBox.boundingRect(),Scalar(0,0,255),3);
                        }

                        if( backprojMode )
                        	cvtColor( backproj, image, COLOR_GRAY2BGR );
			else if( trackObject < 0 )
                        	paused = false;

                        if( selectObject && selection.width > 0 && selection.height > 0 )
                        {
                        	Mat roi(image, selection);
	                        bitwise_not(roi, roi);
                        }

			if (trackObject>=0)
			{
				imshow( "CamShift Demo", image );
                        	waitKey(10);
			}
                        char c = (char)waitKey(30);
                        if( c == 27 )
                        	exit(0);
                        switch(c)
                        {
	                        case 'b':
          		                      backprojMode = !backprojMode;
                                              break;
                                case 'c':
                                              trackObject = 0;
                                              histimg = Scalar::all(0);
                                              break;
                                case 'h':
                                              showHist = !showHist;
                                              if( !showHist )
                        	                      destroyWindow( "Histogram" );
					      else
                                              	      namedWindow( "Histogram", 1 );
                                              break;
                                case 'p':
                                	      paused = !paused;
                                              break;
                                default:
                                              ;
                        }

			if(!paused)
                        {
			       estPixel = trackBox.boundingRect().width;
				if(estPixel!=1)
				{
			      		z = hgtEst(estPixel,0.0025,3.31,177.8);
					x = persProj(trackBox.boundingRect().x+((trackBox.boundingRect().width)/2.0), z, 1325.55);
            				y = persProj(trackBox.boundingRect().y+((trackBox.boundingRect().height)/2.0),z, 1325.55);

        			}
				else
        			{
					x=0;y=0;z=0;
				}
				vision_msg.position[0] = x;
        			vision_msg.position[1] = y;
        			vision_msg.position[2] = z;

        			lcm1.publish("vision_state", &vision_msg);

			       cap >> frame;
 	                       if( frame.empty() )
        	                       break;
                        }

		}
	}	
}

