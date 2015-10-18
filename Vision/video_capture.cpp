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
#include "boost/date_time.hpp"
#include "boost/thread.hpp"
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <signal.h>
#include <sys/time.h>
#include <opencv2/core/types_c.h>
using namespace std;
using namespace boost::posix_time;
using namespace cv;

struct compos{
int64_t curTime;
cv::Mat Frame;
};

//Boost based circular buffer
template <typename T>
class circ_buffer : private boost::noncopyable
{
public:
    typedef boost::mutex::scoped_lock lock;
    circ_buffer() {}
    circ_buffer(int n) {cb.set_capacity(n);}
    void send (T imdata) {
        lock lk(monitor);
        cb.push_back(imdata);
        buffer_not_empty.notify_one();
    }
    T receive() {
        lock lk(monitor);
        while (cb.empty())
            buffer_not_empty.wait(lk);
        T imdata = cb.front();
        cb.pop_front();
        return imdata;
    }
    void clear() {
        lock lk(monitor);
        cb.clear();
    }
    int size() {
        lock lk(monitor);
        return cb.size();
    }
    void set_capacity(int capacity) {
        lock lk(monitor);
        cb.set_capacity(capacity);
    }
    bool check_state()	{
		return cb.empty();
    }
private:
    boost::condition buffer_not_empty;
    boost::mutex monitor;
    boost::circular_buffer<T> cb;
};
circ_buffer<compos> cb(1000);

//Global variables
uint8_t *buffer;
bool camActive = false; 
sig_atomic_t CLOSE_FLAG=1;

//Interrupt handler
void my_handler (int param)
{
  CLOSE_FLAG = 0;
}

//Main Function to query driver
static int xioctl(int fd, int request, void *arg)
{
        int r;
 
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
 
        return r;
}
 
//Query camera capabilites
int print_caps(int fd,int width, int height)
{
        struct v4l2_capability caps = {};
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
        {
                perror("Querying Capabilities");
                return 1;
        }
 
        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d.%d\n"
                "  Capabilities: %08x\n",
                caps.driver,
                caps.card,
                caps.bus_info,
                (caps.version>>16)&&0xff,
                (caps.version>>24)&&0xff,
                caps.capabilities);
 
 
        struct v4l2_cropcap cropcap = {0};
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
        {
                perror("Querying Cropping Capabilities");
                return 1;
        }
 
        printf( "Camera Cropping:\n"
                "  Bounds: %dx%d+%d+%d\n"
                "  Default: %dx%d+%d+%d\n"
                "  Aspect: %d/%d\n",
                cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
                cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
                cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
 
        int support_grbg10 = 0;
 
        struct v4l2_fmtdesc fmtdesc = {0};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        char fourcc[5] = {0};
        char c, e;
       cout<<"  FMT : CE Desc\n--------------------\n";
        while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
        {
                strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
                if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
                    support_grbg10 = 1;
                c = fmtdesc.flags & 1? 'C' : ' ';
                e = fmtdesc.flags & 2? 'E' : ' ';
                printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
                fmtdesc.index++;
        }
        /*
        if (!support_grbg10)
        {
            printf("Doesn't support GRBG10.\n");
            return 1;
        }*/
 
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        {
            perror("Setting Pixel Format");
            return 1;
        }
 
        strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
        printf( "Selected Camera Mode:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  PixFmt: %s\n"
                "  Field: %d\n",
                fmt.fmt.pix.width,
                fmt.fmt.pix.height,
                fourcc,
                fmt.fmt.pix.field);
        return 0;
}
 
int init_mmap(int fd)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 100; //Doesnt make a difference changing to high or low value
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
 
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }
 
    buffer = (uint8_t*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);
 
    return 0;
}
 
//Thread to capture image from camera buffer
int capture_image(int fd)
{
	int i=1;
	struct compos data;
	struct timeval tv1;
	cv::Mat m;
	while(CLOSE_FLAG)
	{
	//Minor delay due to function
   		gettimeofday(&tv1,NULL);
    	struct v4l2_buffer buf = {0};
    	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    	buf.memory = V4L2_MEMORY_MMAP;
    	buf.index = 0;
    	if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    	{
    	    perror("Query Buffer");
    	    return 1;
    	}
 	
    	if (!camActive){
    	if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    	{
    	    perror("Start Capture");
    	    return 1;
    	}
    	camActive = true;
    	}
    	fd_set fds;
    	FD_ZERO(&fds);
    	FD_SET(fd, &fds);
    	struct timeval tv = {0};
    	tv.tv_sec = 2;
    	int r = select(fd+1, &fds, NULL, NULL, &tv);
    	if(-1 == r)
    	{
    	    perror("Waiting for Frame");
    	    return 1;
    	}
 	
    	if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    	{
    	    perror("Retrieving Frame");
    	    return 1;
    	}
    	
    	CvMat cvmat = cvMat(1080,1920, CV_8UC3, (void*)buffer);
    	data.curTime = (int64_t)tv1.tv_sec*1000000 + tv1.tv_usec; 
	CvMat* src = &cvmat;
	m = cv::cvarrToMat(&cvmat);
//	data.Frame = cv::cvarrToMat(&cvmat,true);
	data.Frame = imdecode(cv::InputArray(m), 1);
//	data.Frame = m;
//	namedWindow("M",1);
//	imshow("M",m);
//	waitKey(10);
//	cout<<data.Frame[1][1][1];
//	waitKey(0);
	
	cb.send(data);

	cout<<++i<<endl;	
		} 
	cout<<"\nClosing capture thread\n";
   	return 0;
}
 
//Asynchronos thread to save image
int save_image(int fd, VideoWriter* writer)
{
	int i=1;
	struct compos output;
	ofstream log;
	log.open("log.txt",ios::out|ios::app);

	while(CLOSE_FLAG)
	{
 		while(cb.check_state()){}
 		output = cb.receive();
		writer->write(output.Frame);
		log<<output.curTime <<" Frame: "<<i<<endl;
		i+=1;
 	}
 	while(!cb.check_state())	
 	{
 		output = cb.receive();
		writer->write(output.Frame);
		log<<output.curTime<<" Frame: "<<i<<endl;
		i+=1;
	}
	log.close();
	cout<<"\nClosing save thread\n";
	return 0;
}

//Camera setup function
int setupGUI()
{

	VideoCapture stream1(1);   //0 is the id of video device.0 if you have only one camera.
 
	if (!stream1.isOpened()) 
	{ 
		cout << "cannot open camera";
	}
 
	while(CLOSE_FLAG)
	{
	Mat cameraFrame;
	stream1.read(cameraFrame);
	imshow("cam", cameraFrame);
	if (waitKey(30) >= 0)
		break;
	}	
	CLOSE_FLAG=1;
	destroyAllWindows();
	stream1.release();
	return 0;
}

//Main function
int main()
{	
	int fd,width = 1920, height = 1080;
	double framerate = 30.0;
    // Remove any existing log file    
	remove("log.txt");
		
 	VideoWriter writer = VideoWriter("output.avi",CV_FOURCC('M','J','P','G'),framerate,cvSize(width,height),1);
 	
 	//Setup interrupt handler	
	void(*prev_handler)(int);
	prev_handler = signal (SIGINT,my_handler);
	
    //Camera setup function  
    setupGUI();
    
    //Open camera device
	fd = open("/dev/video1", O_RDWR);
    if (fd == -1)
    {
		perror("Opening video device");
        return 1;
    }

    if(print_caps(fd,width,height))
        return 1;
        
    if(init_mmap(fd))
        return 1;

   	boost::thread captureThread(capture_image,fd);	 
   	boost::thread saveThread(save_image,fd,&writer);       

	while(CLOSE_FLAG){}
	
	//Ensure synchronized close of threads
	captureThread.join();
	saveThread.join();
	cout<<"\nClosing main thread\n";
	close(fd);

    return 0;
}
