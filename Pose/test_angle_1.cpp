#include "AngleSolver.hpp"
#include "../Armor/ArmorDetector.h"
//#include "Predictor.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#define JHAngletest
#define DEBUG
using namespace cv;
using namespace std;

void onSharpChange(int sharp, void* capture)
{
	((cv::VideoCapture*)capture)->set(CV_CAP_PROP_SHARPNESS, sharp - 5);
}
void onExpChange(int exposure, void* capture)
{
	((cv::VideoCapture*)capture)->set(CV_CAP_PROP_EXPOSURE, exposure - 10);
}

int main()
{

	cv::VideoCapture capture(1);//open default camera

								/*cv::VideoWriter write;
								string outfile = "E://samples.avi";
								double r = capture.get(CV_CAP_PROP_FPS);*/

	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	cv::Size fram_size = cv::Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	//write.open(outfile, -1, r, fram_size, true);

	//size_t frame_cnt = 0;

	int exposure = 0, sharpness = 0;
	cv::namedWindow("control panel");
	cv::createTrackbar("exposure", "control panel", &exposure, 10, onExpChange, &capture);
	cv::createTrackbar("sharpness", "control panel", &sharpness, 10, onSharpChange, &capture);
	//capture.set(CV_CAP_PROP_EXPOSURE, -7);

	rm::ArmorParam param;
	rm::ArmorDetector armorDetector(param);
	/*	VideoCapture capture;
	String PathName1 = "E:/roborobo/Robomaster_sample/official/步兵素材蓝车右后对角-ev-0.MOV";
	String PathName2 = "E:/roborobo/old_samples/video_pieces/比赛视频1.mp4";
	String PathName3 = "E:/roborobo/Robomaster_sample/Samples/car1.mp4";
	capture.open(PathName3);*/
	int cam_number = 4;
	AngleSolverParam param2;
	param2.readFile(cam_number);
	AngleSolver angleSolver(param2);
	AngleFlag solve_result;

	Point2f targetCenter;
	std::vector<cv::Point2f> targetPoints;
	std::vector<cv::Point2f> targetEmpty;
	for (int tt = 0; tt <= 3; tt++)
		targetEmpty.push_back(cv::Point2f(0.0, 0.0));

	//Predictor predictor;
	//int frame_num = 0;

	if (!capture.isOpened())
	{
		std::cerr << "can't open camera" << std::endl;
		system("pause");
		return -1;
	}

	while (1)
	{
		Mat frame;
		capture >> frame;
		//waitKey(0);
		if (frame.empty()) continue;



		armorDetector.loadImg(frame);
		armorDetector.detectArmors();
		if (angleSolver.showAlgorithm() == 0)
		{
			targetCenter = armorDetector.getTargetArmorCenter();
			if (targetCenter == Point2f(0.0, 0.0)) {
				std::cout << "no armor detected" << std::endl; continue;
			}
			angleSolver.setTarget(targetCenter, 1);
		}
		if (angleSolver.showAlgorithm() == 1)
		{
			targetPoints = armorDetector.getTargetArmor4Points();
			if (targetPoints == targetEmpty)
			{
				std::cout << "no armor detected" << std::endl;
				continue;
			}
			angleSolver.setTarget(targetPoints, 1);
		}

#ifdef DEBUG
		angleSolver.showPoints2dOfArmor();
#endif // DEBUG

		/*
		if (!(angleSolver.isTargetGiven()))
		{
		std::cout << "no armor detected." << std::endl;
		continue;
		}
		*/

		solve_result = angleSolver.solve();

#ifdef DEBUG

		std::cout << solve_result << std::endl;
		//angleSolver.showTvec();
		angleSolver.showAngle();
		angleSolver.showEDistance();
#endif // DEBUG
	}

	system("pause");
	return 0;
}
/*
Anglesolver类的使用说明：
使用步骤：
1.创建配套的参数类AngleSolverParam 实例如下：
int cam_number = 2;
AngleSolverParam param2;
param2.readFile(cam_number);
AngleFlag solve_result;
2.创建Anglesolver类 实例如下：
AngleSolver angleSolver(param2);
3.设置相关参数 实例如下：
angleSolver.setResolution(imagesize);
angleSolver.setUserType(1);
angleSolver.setBulletSpeed();
4.传入识别点 若识别为空不解算角度 实例如下：
if (angleSolver.showAlgorithm() == 0)
{
targetCenter = armorDetector.getTargetArmorCenter();
if (targetCenter == Point2f(0.0, 0.0)) {
std::cout << "no armor detected" << std::endl;
continue;
}
angleSolver.setTarget(targetCenter, 1);
}
if (angleSolver.showAlgorithm() == 1)
{
targetPoints = armorDetector.getTargetArmor4Points();
if (targetPoints == targetEmpty)
{
std::cout << "no armor detected" << std::endl;
continue;
}
angleSolver.setTarget(targetPoints, 1);
}
if (angleSolver.showAlgorithm() == 2)......angleSolver.setTarget(runePoints);
5.解算角度 实例如下：
solve_result = angleSolver.solve();
(5.5.如有需要 可以开启重力补偿 但因为不同的车尺寸多有不同 可能会造成不准确
angleSolver.getCompensateAngle()；
)
6.获得结果 实例如下：
alpha=angleSolver.getAngle();
dis=angleSolver.getDistance();
*/
