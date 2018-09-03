#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>

#include <libv4l2.h>
#include <linux/videodev2.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

#include"../General/General.h"
#include"../Driver/RMVideoCapture.hpp"

using namespace cv;
using namespace std;
using namespace rm;

/*
*	To prevent camera from dying!
*/
bool quit_flag = 0;
void signal_handler(int)
{
    quit_flag = true;
}

void init_signals(void)
{
    quit_flag = false;
    struct sigaction sigact;
    sigact.sa_handler = signal_handler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT,&sigact,(struct sigaction*)NULL);
}


int main()
{
	init_signals();

	/**
	*	Inintialize camera
	*	TODO: set sharpness
	*/
//    RMVideoCapture capture(1, 2);
    RMVideoCapture capture;
    capture.open(1,2);
    if(!capture.isOpened())
    {
        return -1;
    }

    capture.setVideoFormat(1280, 720, true);
    capture.setExposureTime(100);
    capture.setFPS(60);
	capture.startStream();
	capture.info();

//	const cv::Size frameSize = capture.getResolution();

//	VideoWriter writer("/home/ubuntu/RoboVideo/"+to_string(time(NULL)) + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, frameSize);


	Mat frame;
	size_t frameCnt = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
	for(;;)
	{
		if (quit_flag) 
		{
			return 0;
		}
		

		capture>>frame;
		if(frame.empty())
		{
			cout<<"image failed"<<endl;
			continue;
		}
		frameCnt++;

        auto t2 = chrono::high_resolution_clock::now();
        cout << (chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)).count() << "ms" << std::endl;
        t1 = t2;


		//writer << frame;

        imshow("frame",frame);
        if(waitKey(1)=='q')
        {
            break;
        }
	}

	return 0;
}

