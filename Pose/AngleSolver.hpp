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
	cv::Mat CAM_MATRIX;			// 相机内参
	cv::Mat DISTORTION_COEFF;	//相机畸变参数
	//三维坐标的单位是毫米 
	static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_BIG;
	static std::vector<cv::Point3f> POINT_3D_OF_ARMOR_SMALL;
	static std::vector<cv::Point3f> POINT_3D_OF_RUNE;
	double Y_DISTANCE_BETWEEN_GUN_AND_CAM = 0;//If gun is under the cam, this variable is positive.
	cv::Size CAM_SIZE = cv::Size(1920, 1080);
	double GUARD_HIGHT = 4000;

	/*
	* @brief 设置相机参数
	* @param Input int id 1-4对应相应摄像头，5对应官方图传，0对应测试用双目摄像头左目
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
		ANGLE_ERROR = 0,                //计算错误
		ONLY_ANGLES = 1,				//只能得到角度信息
		TOO_FAR = 2,					//距离太远，距离可能不准确
		ANGLES_AND_DISTANCE = 3			//角度和距离都准确
	};

	
	/*
	* @brief 设置二维装甲板点,可以接受中心坐标也可以接受四角坐标
	* @param Input armorPoints/centerPoint
	*/
	void setTarget(const std::vector<cv::Point2f> objectPoints, int objectType);  //设置二维装甲板或神符点
	void setTarget(const cv::Point2f centerPoint, int objectType);//赋值中心点坐标
	//void setTarget(const std::vector<cv::Point2f> runePoints);  //设置二维神符点
															

	/*
	* @brief 根据选定的算法类型解算角度
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
	* @brief 设置给出图像分辨率
	*/
	void setResolution(const cv::Size2i& image_resolution);

	/*
	* @brief 设置运行程序的车辆种类，0代表哨兵，1代表步兵，2代表英雄；
	*/
	void setUserType(int usertype);

	/*
	* @brief 设置敌人类型，0代表小装甲板，1代表大装甲板；
	*/
	void setEnemyType(int enemytype);

	/*
	* @brief 设置子弹速度；
	*/
	void setBulletSpeed(int bulletSpeed);

	/*
	* @brief 显示解算的角度结果
	*/
	const cv::Vec2f getAngle();

	/*
	* @brief 进行重力补偿
	*/
	//const cv::Vec2f getCompensateAngle();

    //const cv::Vec2f getPredictedAngle();

	/*
	* @brief 显示解算的距离结果，仅用PNP时
	*/
    double getDistance();

#ifdef DEBUG
	/*
	* @brief展示装甲板四角的二维坐标
	*/
	void showPoints2dOfArmor();

	/*
	* @brief 展示输出矩阵
	*/
	void showTvec();

	/*
	* @brief 输出角度和欧氏距离
	*/
	void showEDistance();

	/*
	* @brief 输出装甲板中心坐标
	*/
	void showcenter_of_armor();

	/*
	* @brief 显示解算角度结果
	*/
	void showAngle();

	/*
	* @brief 返回算法信息
	*/
	int showAlgorithm();
#endif // DEBUG

private:
	AngleSolverParam _params;
	cv::Mat _rVec = cv::Mat::zeros(3, 1, CV_64FC1);//初始化输出矩阵 
	cv::Mat _tVec = cv::Mat::zeros(3, 1, CV_64FC1);//初始化输出矩阵
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
	int user_type = 1;//运行程序的车辆种类，0代表哨兵，1代表步兵，2代表英雄；
	int enemy_type = 1;//敌人类型，0代表小装甲板，1代表大装甲板；
	double _bullet_speed = 22000;
	double _rune_compensated_angle = 0;
	int is_shooting_rune = 0;
	cv::Mat _cam_instant_matrix;// 相机内参
    //std::vector<cv::Point2f> _x_time_points;
	//std::vector<cv::Point2f> _y_time_points;
	//int framecount=0;
};

}
