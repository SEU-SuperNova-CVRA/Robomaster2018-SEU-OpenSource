#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include<math.h>

#include<AngleSolver.hpp>
#include"ArmorDetector.h"

using namespace cv;
using namespace std;
using namespace rm;

Vec3b HSV2BGR(const Vec3b& hsv)
{
	auto h = hsv(0);
	auto s = (double)hsv(1)/255.0;
	auto v = (double)hsv(2)/255.0;
	double b = 0;
	double g = 0;
	double r = 0;

	int flag = h / 60;
	double f = (double)h / 60.0 - flag;
	double p = v * (1 - s);
	double q = v * (1 - f * s);
	double t = v * (1 - (1 - f)*s);

	switch(flag)
	{
	case 0:
		b = p;
		g = t;
		r = v;
		break;
	case 1:
		b = p;
		g = v;
		r = q;
		break;
	case 2:
		b = t;
		g = v;
		r = p;
		break;
	case 3:
		b = v;
		g = q;
		r = p;
		break;
	case 4:
		b = v;
		g = p;
		r = t;
		break;
	case 5:
		b = q;
		g = p;
		r = v;
		break;
	default:
		break;
	}

	Vec3b bgr;
	int blue = int(b * 255);
	bgr(0) = (blue > 255) ? 255 : blue;
	bgr(0) = (blue < 0) ? 0 : bgr(0);

	int green = int(g * 255);
	bgr(1) = (green > 255) ? 255 : green;
	bgr(1) = (green < 0) ? 0 : bgr(1);

	int red = int(r * 255);
	bgr(2) = (red > 255) ? 255 : red;
	bgr(2) = (red < 0) ? 0 : bgr(2);

	return bgr;
}

int main()
{

	/**
	*	Inintialize camera
	*/
	const string fileName = "447941453";
	VideoCapture capture("C:/Users/Binyan/Documents/School Works/Robomaster/装甲板样本/" + fileName + ".avi");


	/*
	*	Initialize angle solver
	*/
	AngleSolverParam angleParam;
	angleParam.readFile(4);
	rm::AngleSolver angleSolver(angleParam);
	angleSolver.setResolution(Size(1280, 720));

	int angleFlag;
	Vec2f targetAngle;
	double targetDistance;


	/*
	*	Initialize armor detector
	*/
	ArmorParam armorParam;
	ArmorDetector armorDetector(armorParam);
	armorDetector.setEnemyColor(BLUE);

	int armorFlag;
	int armorType;
	std::vector<cv::Point2f> armorVertex;


	/*  */
	Mat map = Mat::zeros(500, 500, CV_8UC3);
	Vec3b color(255, 255, 255);
	//color = HSV2BGR(color);
	size_t mapIdx = 0;
	size_t pointsCnt = 0;

	Mat frame;
	size_t frameCnt = 0;
	for(;;)
	{
		capture >> frame;
		if(frame.empty())
		{
			continue;
		}
		frameCnt++;

		armorDetector.loadImg(frame);
		armorFlag = armorDetector.detect();
		switch(armorFlag)
		{
		case ArmorDetector::ARMOR_LOCAL:
			pointsCnt++;
			break;
		case ArmorDetector::ARMOR_GLOBAL:
			pointsCnt++;
			//color = Vec3b(rand() & 360, 255, 255);
			//color = HSV2BGR(color);
			break;
		default:
			//if(pointsCnt > 10)
			//{
			//	mapIdx++;
			//	imwrite(fileName+"_map_" + to_string(mapIdx) + ".jpg", map);
			//}
			//destroyWindow("map" + to_string(mapIdx));
			//map = Mat::zeros(500, 500, CV_8UC3);
			//pointsCnt = 0;
			//color = Vec3b(rand() & 360, 225, 255);
			//color = HSV2BGR(color);
			continue;
		}
		armorVertex = armorDetector.getArmorVertex();
		armorType = armorDetector.getArmorType();

		angleSolver.setTarget(armorVertex, armorType);
		angleFlag = angleSolver.solve();
		if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
		{
			targetAngle = angleSolver.getAngle();
			cout << targetAngle << endl;

			angleSolver.compensateOffset();
			targetAngle = angleSolver.getAngle();
			cout << targetAngle << endl;

			angleSolver.compensateGravity();
			targetAngle = angleSolver.getAngle();
			cout << targetAngle << endl;

			targetDistance = angleSolver.getDistance();
			cout << targetDistance << endl;

			cout << endl;

			auto x = targetDistance * std::sin(targetAngle(0) / 180 * CV_PI) / 10;
			auto y = targetDistance * std::cos(targetAngle(0) / 180 * CV_PI) / 10;

			//cout << x << " " << y << endl;

			auto x1 = x + 250;
			auto y1 = 500 - y;

			if(x1 > 499) x1 = 499;
			else if(x1 < 0)x1 = 0;
			if(y1 > 499)y1 = 499;
			else if(y1 < 0)y1 = 0;

			map.at<Vec3b>(y1, x1) = color;

			imshow("map", map);
			waitKey(0);
		}

		//armorDetector.showDebugImg();
		//waitKey();
	}

	return 0;
}

