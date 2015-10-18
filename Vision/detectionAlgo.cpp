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
#include <fstream>
#include <iostream>
#include <ctype.h>
#include <iostream>
#include <ctime>
#include "../include/util.h"
#include "../include/filter_util.h"
#include <lcm/lcm-cpp.hpp>
#include "../lcmtypes/vision_state_t.hpp"
#define NUM_SAMPLES_MED 5
#define NUM_SAMPLES_AVG 5

lcm::LCM lcm1;
vision_state_t vision_msg;

using namespace std;
using namespace cv;



int CAM_VMIN = 10,CAM_VMAX = 256, CAM_SMIN =50 , selectObject = false,trackObject = 0;
bool backprojMode = false;
bool showHist = true;
Point origin;
Rect selection;

int FRAME_W = 1920;
int FRAME_H = 1080;
int frame_number;
Mat image,frame,bgframe;

int64_t time_diff,start_time,end_time;

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

//position filters
double pos_x[NUM_SAMPLES_MED], pos_y[NUM_SAMPLES_MED], pos_z[NUM_SAMPLES_MED];
double pos_x_med[NUM_SAMPLES_AVG], pos_y_med[NUM_SAMPLES_AVG], pos_z_med[NUM_SAMPLES_AVG];
double pos_x_avg[NUM_SAMPLES_AVG], pos_y_avg[NUM_SAMPLES_AVG], pos_z_avg[NUM_SAMPLES_AVG];

//velocity filters

double vel_x[NUM_SAMPLES_MED], vel_y[NUM_SAMPLES_MED], vel_z[NUM_SAMPLES_MED];
double vel_x_med[NUM_SAMPLES_AVG], vel_y_med[NUM_SAMPLES_AVG], vel_z_med[NUM_SAMPLES_AVG];
double vel_x_avg[NUM_SAMPLES_AVG], vel_y_avg[NUM_SAMPLES_AVG], vel_z_avg[NUM_SAMPLES_AVG];

double vx, vy, vz;
double x_prev, y_prev, z_prev;


int main()

{
  if(!lcm1.good()) return 1;

  	VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	cap >> bgframe;

	cap >> frame;

	start_time = utime_now();
		
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
	namedWindow( "CamShift Demo", CV_WINDOW_AUTOSIZE );
	setMouseCallback( "CamShift Demo", onMouse, 0 );

	while(!frame.empty())
	{
		
		RotatedRect trackBox;
                if( !paused )
                {

			frame = abs(bgframe - frame);
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

			
				imshow( "CamShift Demo", image );
                        	waitKey(10);
			
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
			      		z = hgtEst(estPixel,0.0025,3.31,140);
					x = persProj(trackBox.boundingRect().x+((trackBox.boundingRect().width)/2.0), z, 1325.55);
            				y = persProj(trackBox.boundingRect().y+((trackBox.boundingRect().height)/2.0),z, 1325.55);

        			}
				else
        			{
					x=0;y=0;z=0;
				}
			        cap >> frame;

            addArr(pos_x, x, NUM_SAMPLES_MED); 
            addArr(pos_y, y, NUM_SAMPLES_MED);
            addArr(pos_z, z, NUM_SAMPLES_MED);

            addArr(pos_x_med, median(pos_x, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(pos_y_med, median(pos_y, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(pos_z_med, median(pos_z, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);

            addArr(pos_x_avg, average(pos_x_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            addArr(pos_y_avg, average(pos_y_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            addArr(pos_z_avg, average(pos_z_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);


            vision_msg.position[0] = pos_x_avg[NUM_SAMPLES_AVG - 1];
            vision_msg.position[1] = pos_y_avg[NUM_SAMPLES_AVG - 1];
            vision_msg.position[2] = pos_z_avg[NUM_SAMPLES_AVG - 1];


				end_time = utime_now();
				time_diff = end_time - start_time;
        // cout<<time_diff<<endl;
				start_time = end_time;
				
				vx = (vision_msg.position[0] - x_prev)/time_diff*1000;
				vy = (vision_msg.position[1] - y_prev)/time_diff*1000;
				vz = (vision_msg.position[2] - z_prev)/time_diff*1000;

        addArr(vel_x, vx, NUM_SAMPLES_MED);
        addArr(vel_x_med, median(vel_x, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
        addArr(vel_x_avg, average(vel_x_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

        addArr(vel_y, vy, NUM_SAMPLES_MED);
        addArr(vel_y_med, median(vel_y, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
        addArr(vel_y_avg, average(vel_y_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

        addArr(vel_z, vz, NUM_SAMPLES_MED);
        addArr(vel_z_med, median(vel_z, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
        addArr(vel_z_avg, average(vel_z_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

        vision_msg.velocity[0] = vx;
        vision_msg.velocity[1] = vy;
        vision_msg.velocity[2] = vz;

        
  			lcm1.publish("vision state", &vision_msg);

        if( frame.empty())

  	                       break;

        }

        x_prev = vision_msg.position[0];
        y_prev = vision_msg.position[1];
        z_prev = vision_msg.position[2];



		}
	}	
}

