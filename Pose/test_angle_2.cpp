#include <unistd.h>
#include <time.h>
#include <libv4l2.h>
#include <signal.h>
#include <linux/videodev2.h>

#include <fcntl.h>
#include <errno.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <chrono>

#include "../General/General.h"
#include"../Driver/RMVideoCapture.hpp"
#include "../Rune/Rune.h"
#include "../Pose/AngleSolver.hpp"
#include "../Serials/Serial.h"

#define DEBUG

using namespace cv;
using namespace std;
using namespace rm;

bool quit_flag = 0;
Serial serial;
uint8_t frame_cnt = 0;

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



void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	AngleSolver* agPtr = (AngleSolver*)ustc;
	if ((event == CV_EVENT_LBUTTONDOWN))//鼠标左键按下事件发生    
	{
		cout<<x<<" "<<y<<endl;
//        agPtr->setTarget(vector<Point2f>{Point2f(x-10, y-10),
//                                         Point2f(x+10, y-10),
//                                         Point2f(x+10,y+10),
//                                         Point2f(x-10,y+10),},1);
        agPtr->setTarget(Point2f(x, y), 1);
		agPtr->solve();

        Vec2f angles = agPtr->getAngle();
		cout<<angles[0]<<" "<<angles[1]<<endl;

        ControlData ctrl;
        ctrl.frame_seq = 0.0f;
        ctrl.shoot_mode = Serial::NO_FIRE | Serial::HIGH_SPEED;
        ctrl.pitch_dev = angles[1] ;
        ctrl.yaw_dev = angles[0] ;

        serial.tryControl(ctrl, chrono::milliseconds(3));
	}
}


int main()
{

	init_signals();

    /*
	*	Inintialize camera, set exposuer by V4L2, but finally use OpenCV interface
	*	TODO: set sharpness
	*/
    RMVideoCapture capture;
    capture.open(0,2);
    if(!capture.isOpened())
    {
        return -1;
    }
    capture.setVideoFormat(640, 480, true);
    capture.setExposureTime(200);
    capture.setFPS(60);
    capture.startStream();
    capture.info();

    //inintialize serial
    serial.openPort();
    serial.setDebug(false);
    int self_color;
    while(serial.setup(self_color) != Serial::OJBK)
    {
        sleep(1);
    }
    cout << "I am " << (self_color == rm::BLUE ? "blue" : "red") << "." << endl;

	Mat frame;



	if (!capture.isOpened())
	{
		std::cerr << "can't open camera" << std::endl;
		system("pause");
		return -1;
	}


    //Initialize angle solveCompensater
    AngleSolver solverPtr;
    AngleSolverParam angleParam;
    angleParam.readFile(8);
    solverPtr.init(angleParam);
    solverPtr.setResolution(capture.getResolution());

		

	namedWindow("src");
    setMouseCallback("src", on_mouse, (void*)(&solverPtr));  //注册鼠标相应回调函数

	while (1)
	{
		if (quit_flag)
		{
			return 0;
		}

        serial.tryRecord(frame_cnt, chrono::milliseconds(3));
		capture >> frame;
        cvex::drawCrossing(frame, frame, Point2f(320,240), cvex::GREEN);
		
		if (frame.empty()) continue;

		cv::imshow("src", frame);	

		waitKey(1);
	}

	system("pause");
	return 0;
}
