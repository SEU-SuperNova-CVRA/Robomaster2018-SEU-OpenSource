
#ifndef _MSC_VER
#include <unistd.h>
#include <sys/time.h>
#include <libv4l2.h>
#include <signal.h>
#include <linux/videodev2.h>
#endif

#include <fcntl.h>
#include <errno.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <array>
#include <vector>

#include "../Pose/AngleSolver.hpp"
#include "../Serials/Chariot.h"

#include "../Rune/Rune.h"
#include"../General/General.h"


using namespace cv;
using namespace std;
using namespace rm;

#define DEBUG
//#define SHOW_TARGET
#define SHOOT_AND_SLEEP

#ifdef _MSC_VER
void onSharpChange(int sharp, void* capture)
{
	((cv::VideoCapture*)capture)->set(CV_CAP_PROP_SHARPNESS, sharp - 5);
}
void onExpChange(int exposure, void* capture)
{
	((cv::VideoCapture*)capture)->set(CV_CAP_PROP_EXPOSURE, exposure - 10);
}
#else // _MSC_VER
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
#endif
int main()
{
	/**
	*	Inintialize camera, set exposuer by V4L2, but finally use OpenCV interface
	*	TODO: set sharpness
	*/
#ifndef _MSC_VER
	//Set exposure
	{
		char CamearDir[16];
		sprintf(CamearDir, "/dev/video%d", 0); // Device ID is 0
		int camraId = v4l2_open(CamearDir, O_RDWR | O_NONBLOCK);
		if(camraId == -1)
		{
			cout << "Camera V4L2 device open failed." << endl;
		}
		int exposure = 2000;
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
#endif // !_MSC_VER

	//VideoCapture capture("E:/Shool Works/项目/RM 样本/符/bigBuff.avi");
	//VideoCapture capture("Samples/SmallRune.avi");
	//VideoCapture capture("/home/nvidia/bigBuff.avi");
	VideoCapture capture("Samples/2.avi");
	//VideoCapture capture(0);
	if(!capture.isOpened())
	{
		std::cerr << "can't open camera" << std::endl;
		return -1;
	}

#ifdef _MSC_VER
	//int exposure = 0, sharpness = 0;
	//cv::namedWindow("control panel");
	//cv::createTrackbar("exposure", "control panel", &exposure, 10, onExpChange, &capture);
	//cv::createTrackbar("sharpness", "control panel", &sharpness, 10, onSharpChange, &capture);
	//capture.set(CV_CAP_PROP_EXPOSURE, -7);
#endif // _MSC_VER

	const cv::Size frameSize = cv::Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));

	
	/*
	*	Initialize armor detector
	*	TODO: set bullet speed
	*/
	rm::RuneDetector& runeDetector = rm::RuneDetector::getInstance();
	runeDetector.init(rm::MINI_RUNE, rm::RuneParam());
	std::array<cv::Point2f, 9> runeCenters;
	std::array<int, 9> runeNumbers;
	std::vector<int> ledNumbers;
	int runeFlag;
	int shootFlag;


	/*
	*	The main loop
	*/
	Mat frame;
	size_t frameCnt = 0;
	while(capture.read(frame))
	while(1)
	{
#ifndef _MSC_VER
		  if (quit_flag)
			return 0;
#endif

#ifdef DEBUG
		  frameCnt++;
		  std::cout << "frame count: " << frameCnt << std::endl;
#endif // DEBUG


		//frame = imread("1111//"+std::to_string(frameCnt) + ".jpg");
		//frame = imread("1111//60.jpg");
		//if(frameCnt < 13)continue;
		//cv::imwrite("1111//" + std::to_string(frameCnt) + ".jpg", frame);

		runeDetector.loadImg(frame);

		//t1 = std::chrono::high_resolution_clock::now();
		
		runeFlag = runeDetector.caculate();

		//t2 = std::chrono::high_resolution_clock::now();
		//time_find_rune = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

		if(runeFlag == rm::RuneDetector::RUNE_NO)
		{
			continue;
		}

		cv::Point2f target = runeDetector.chooseTarget();
#ifdef DEBUG
		std::cout << "target: " << target << std::endl;
#endif

#ifdef SHOW_TARGET
		cv::Mat showtarget;
		cvex::drawCrossing(frame, showtarget, target, cvex::GREEN);
		cv::imshow("target", showtarget);

#endif // SHOW_TARGET
        	shootFlag = runeDetector.getFlag();
#ifdef DEBUG
		cout << "shootFlag:" << shootFlag << endl;
#endif

		if(shootFlag != rm::RuneDetector::SHOOT_NO)
		{
			//TODO: control the gimbal and gun
			cv::circle(frame, target, 5, cvex::CYAN, 3);
			cv::imshow("target",frame);
			int key = waitKey(1);

			//TODO: let the bullet fly
			//打中了就给true

	//            if (cv::waitKey() == 's')
	//            {
	//                runeDetector.isShoot(true);
	//#ifdef DEBUG
	//				std::cout << "let button fly" << std::endl;
	//#endif 
	//				cv::waitKey(1000);
	//            }


		}


	#ifdef DEBUG
		//std::cout << "x is: " << target.x << " y is: " << target.y << std::endl;
		runeCenters = runeDetector.getCenters();
		runeNumbers = runeDetector.getRuneNumbers();
		ledNumbers = runeDetector.getLedNumbers();
		for(int i : ledNumbers)
		{
			std::cout << i << " ";
		}
		std::cout << std::endl;
		for(int i = 1; i <= 9; ++i)
		{
			std::cout << runeNumbers[i - 1] << " ";
			if(i % 3 == 0) std::cout << std::endl;
		}
		std::cout << std::endl;

		cv::waitKey(1);
		std::cout << std::endl;
	#endif // DEBUG

		//std::cout << time_find_rune.count() << "ms" << std::endl;

	}

	return 0;
}
