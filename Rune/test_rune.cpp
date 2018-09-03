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
#include "../Rune/Rune.h"
#include "../Pose/AngleSolver.hpp"
#include "../Serials/Chariot.h"

using namespace cv;
using namespace std;
using namespace rm;


#define DEBUG
//#define SHOW_TARGET
#define SHOOT_AND_SLEEP

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
	*	Inintialize camera, set exposuer by V4L2, but finally use OpenCV interface
	*	TODO: set sharpness
	*/
	{
		char CamearDir[16];
		sprintf(CamearDir, "/dev/video%d", 0); // Device ID is 0
		int camraId = v4l2_open(CamearDir, O_RDWR | O_NONBLOCK);
		if(camraId == -1)
		{
			cout << "Camera V4L2 device open failed." << endl;
		}
		int exposure = 1000;
		struct v4l2_control v4l2Control;
		v4l2Control.id = V4L2_CID_EXPOSURE_AUTO;
		v4l2Control.value = 1; // Here set the M/A, 1 is manual, 3 is auto
		if(v4l2_ioctl(camraId, VIDIOC_S_CTRL, &v4l2Control) != 0)
		{
			cout << "Failed to set exposure." << strerror(errno) << endl;
		}
		else
		{
			cout << "Set exposure to: " << exposure << endl;
		}
		v4l2_set_control(camraId, V4L2_CID_EXPOSURE_ABSOLUTE, exposure);
		v4l2_close(camraId);
	}


	//VideoCapture capture("E:/Shool Works/项目/RM 样本/符/bigBuff.avi");
	//VideoCapture capture("Samples/SmallRune.avi");
	//VideoCapture capture("/home/nvidia/bigBuff.avi");
	VideoCapture capture(0);
	if(!capture.isOpened())
	{
		std::cerr << "can't open camera" << std::endl;
		return -1;
	}

	const cv::Size frameSize = cv::Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));

	
	/*
	*	Initialize rune detector
	*	TODO: set bullet speed
	*/
	rm::RuneDetector& runeDetector = rm::RuneDetector::getInstance();
	runeDetector.init(rm::MINI_RUNE, rm::RuneParam());
	int runeFlag;


	/*
	*	Initialize angle solver
	*/
	rm::AngleSolverParam angleParam4;
	angleParam4.readFile(4);
	rm::AngleSolver angleSolver(angleParam4);
	angleSolver.setResolution(frameSize);
	int angleFlag;
	//Vec2f angleK(0.7, 0.8);


	/*
	*	Initialize serials
	*/
	CChariot serials(true, 1, false);
	TControlData ctrl;
	//TFeedBackData feedBack;
	ctrl.fMoveSpeedX = 0.0f;
	ctrl.fHeadAnglePitch = 0;
	ctrl.fHeadAngleYaw = 0;
	ctrl.isShoot = 0;


	/*
	*	The main loop
	*/
	Mat frame;
	size_t frameCnt = 0;
	for(;;)
	{
		//exiting normally or by 'ctrl + c' will not stuck usb socket
		if (quit_flag)
		{
			return 0;
		}
		
		serials.record();

		for(int i=0;i<5;i++)
		{
			capture >> frame;
		}
		
		if(frame.empty()) 
		{
			cout<<"image empty"<<endl;
			continue;
		}
		frameCnt++;
		//std::cout << "frame cou nt: " << frameCnt << std::endl;

		runeDetector.loadImg(frame);	
		runeFlag = runeDetector.caculate();
		if(runeFlag == rm::RuneDetector::RUNE_NO) continue;
        	//cv::Point2f target = runeDetector.getCenters()[4];
		if(runeFlag == rm::RuneDetector::RUNE_NO)
		{
			continue;
		}

		cv::Point2f target = runeDetector.chooseTarget();
#ifdef DEBUG
		std::cout << "target: " << target << std::endl;
#endif
		{
			cv::Mat showtarget;
			cvex::drawCrossing(frame, showtarget, target, cvex::GREEN);
			cv::imshow("target", showtarget);
			
			std::array<int, 9> runeNumbers = runeDetector.getRuneNumbers();
			for(int i = 1; i <= 9; ++i)
			{
				std::cout << runeNumbers[i - 1] << " ";
				if(i % 3 == 0) std::cout << std::endl;
			}
			std::cout << std::endl;
		}


		angleSolver.setTarget(target, 1);
		angleFlag = angleSolver.solve();
		if(angleFlag == rm::AngleSolver::ANGLE_ERROR)
		{
			cout<<"angle solver failed."<<endl;
			continue;
		}
		Vec2f targetAngle = angleSolver.getAngle();

		//targetAngle = targetAngle.mul(angleK);

		//if(abs(targetAngle[0]) < 0.5)targetAngle[0] = 0;
		//if(abs(targetAngle[1]) < 0.5)targetAngle[1] = 0;

		cout<<targetAngle<<endl;

		ctrl.fMoveSpeedX = 0.0f;
		ctrl.fHeadAnglePitch = targetAngle[1];
		ctrl.fHeadAngleYaw   = targetAngle[0];
		ctrl.isShoot = 0;
		serials.Ctrl(&ctrl);

		cv::waitKey(500);
		//wait enough time before the gimbal aims
		
		

		//std::cout << time_find_rune.count() << "ms" << std::endl;
	}

	return 0;
}
