/**************************************************************
MIT License
Copyright (c) 2018 SEU-SuperNova-CVRA
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Authors:	Florentino Zhang, <danteseu@126.com>
**************************************************************/
#pragma once
#include "opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>
#include"../General/General.h"
//#define DEBUG

namespace rm
{

struct AngleSolverParam
{
	cv::Mat CAM_MATRIX;			//Camera Matrix
	cv::Mat DISTORTION_COEFF;	//Distortion matrix
	//the unit of vector is mm
	static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_BIG;
	static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_SMALL;
	static std::vector<cv::Point3f> POINT_3D_OF_RUNE;
	double Y_DISTANCE_BETWEEN_GUN_AND_CAM = 0;//If gun is under the cam, this variable is positive.
	cv::Size CAM_SIZE = cv::Size(1920, 1080);
	double GUARD_HIGHT = 4000;

	/*
	* @brief set the params of camera
	* @param Input int id is the id of camera.you should write a xml including your camera's params.
	*/
	void readFile(const int id);
};

/**
*	solve by PNP, that is, using four points to detect the angle and distance.
*	It's not very useful if the armor is far.If it' far try solve by one point
*/
class AngleSolver
{
public:
    AngleSolver();
    AngleSolver(const AngleSolverParam& AngleSolverParam);

    /*
     *  Initialize with parameters
     */
    void init(const AngleSolverParam& AngleSolverParam);

	enum AngleFlag
	{
		ANGLE_ERROR = 0,                //an error appeared in angle solution
		ONLY_ANGLES = 1,		//only angles is avilable
		TOO_FAR = 2,			//the distance is too far, only angles is avilable
		ANGLES_AND_DISTANCE = 3		//distance and angles are all avilable and correct
	};

	
	/*
	* @brief set the 2D center point or corners of armor, or the center of buff as target 
	* @param Input armorPoints/centerPoint
	*/
	void setTarget(const std::vector<cv::Point2f> objectPoints, int objectType);  //set corner points for PNP
	void setTarget(const cv::Point2f centerPoint, int objectType);//set center points
	//void setTarget(const std::vector<cv::Point2f> runePoints);  //set rune center points
															

	/*
	* @brief slove the angle by selected algorithm
    */
	AngleFlag solve();

	/*
	*      z: direction of the shooter
	*     /
	*  O /______x
	*    |
	*    |
	*    y
	*/
	void compensateOffset();
	void compensateGravity();

	/*
	* @brief tell anglesolver the resolution of used image
	*/
	void setResolution(const cv::Size2i& image_resolution);

	/*
	* @brief set the type of the robot, 0 means gurad, 1 means infantry, 2 means hero
	*/
	void setUserType(int usertype);

	/*
	* @brief set the type of enemy, 0 means enemy has big armor, 1 means small armor.
	*/
	void setEnemyType(int enemytype);

	/*
	* @brief set the speed of bullet
	*/
	void setBulletSpeed(int bulletSpeed);

	/*
	* @brief get the angle solved
	*/
	const cv::Vec2f getAngle();

	/*
	* @brief get the error angle after compensated by the consider of gravity 
	*/
	//const cv::Vec2f getCompensateAngle();

    //const cv::Vec2f getPredictedAngle();

	/*
	* @brief get the distance between the camera and the target
	*/
    double getDistance();

#ifdef DEBUG
	/*
	* @brief show 2d points of armor 
	*/
	void showPoints2dOfArmor();

	/*
	* @brief show tvec 
	*/
	void showTvec();

	/*
	* @brief show distance
	*/
	void showEDistance();

	/*
	* @brief show center of armor
	*/
	void showcenter_of_armor();

	/*
	* @brief show angles
	*/
	void showAngle();

	/*
	* @brief show the information of selected algorithm
	*/
	int showAlgorithm();
#endif // DEBUG

private:
	AngleSolverParam _params;
	cv::Mat _rVec = cv::Mat::zeros(3, 1, CV_64FC1);//init rvec
	cv::Mat _tVec = cv::Mat::zeros(3, 1, CV_64FC1);//init tvec
	std::vector<cv::Point2f> point_2d_of_armor;
	std::vector<cv::Point2f> point_2d_of_rune;
	enum solverAlg
	{
		ONE_POINT = 0,
		PNP4 = 1
	};
	int angle_solver_algorithm = 1;//if 1 ,using PNP solution, if 0 using OnePoint solution,if 2 solving the rune by pnp.
	cv::Point2f centerPoint;
	std::vector<cv::Point2f> target_nothing;
	double _xErr, _yErr, _euclideanDistance;
	cv::Size2i image_size = cv::Size2i(1920, 1080);
	int user_type = 1;
	int enemy_type = 1;
	double _bullet_speed = 22000;
	double _rune_compensated_angle = 0;
	int is_shooting_rune = 0;
	cv::Mat _cam_instant_matrix;// a copy of camera instant matrix
    //std::vector<cv::Point2f> _x_time_points;
	//std::vector<cv::Point2f> _y_time_points;
	//int framecount=0;
};

}
