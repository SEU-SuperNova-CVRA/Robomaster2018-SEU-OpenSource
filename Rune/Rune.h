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

Authors:    Shi Shu, <213162637@seu.edu.cn>
            BinYan Hu
            Su ChenHao
            Cui RenJie
            Sun BoWen
**************************************************************/
#pragma once

#include"../General/singleton.hpp"
#include"../General/General.h"
#include"../General/opencv_extended.h"
#include<opencv2/opencv.hpp>
#include<array>

//#define SHOW_PRETREAT			    /*to show the process of pretreat*/
//#define SHOW_FIND_LED			    /*to show the process of finding led*/
//#define DEBUG_FULL                        /*to show the process of detecting rune*/
//#define DEBUG_NUM                         /*to show the result of caculating numbers*/


#define DEBUG_GET_EVERYTHING                /*to get mumbers of classes*/
//#define CASUAL_SET_NUMBER                 /*to set led numbers and rune numbers while debugging*/

namespace rm
{
/*
 * this class contains all possibly modified parameters which may be used at the playing field
*/
	class RuneParam
	{
	public:
		RuneParam(const int _hsv_h_min = 0,
			const int _hsv_h_max = 34,
			const int _hsv_s_min = 43,
			const int _hsv_s_max = 255,
			const int _hsv_v_min = 150,
			const int _hsv_v_max = 255,
			const int _contour_min_area = 130,
			const float _contour_boundingrect_min_aspect_ratio = 0.25,
			const int _contour_boundingrect_max_aspect_ratio = 4,
			const int _contour_boundingrect_min_area = 200,
			const int _contour_boundingrect_max_area = 3000,
			const float _y_direction_dx_max_width_rate = 0.5,
			const float _y_direction_dy_max_height_rate = 2.5,
			const float _x_direction_dx_min_height_rate = 1.5,
			const float _x_direction_dx_max_height_rate = 3.5,
			const float _x_direction_dy_max_height_rate = 1.0,
			const float _centers_max_sin_theta = 0.30) :
			hsv_h_min(_hsv_h_min),
			hsv_h_max(_hsv_h_max),
			hsv_s_min(_hsv_s_min),
			hsv_s_max(_hsv_s_max),
			hsv_v_min(_hsv_v_min),
			hsv_v_max(_hsv_v_max),
			contour_min_area(_contour_min_area),
			contour_boundingrect_min_aspect_ratio(_contour_boundingrect_min_aspect_ratio),
			contour_boundingrect_max_aspect_ratio(_contour_boundingrect_max_aspect_ratio),
			contour_boundingrect_min_area(_contour_boundingrect_min_area),
			contour_boundingrect_max_area(_contour_boundingrect_max_area),
			y_direction_dx_max_width_rate(_y_direction_dx_max_width_rate),
			y_direction_dy_max_height_rate(_y_direction_dy_max_height_rate),
			x_direction_dx_min_height_rate(_x_direction_dx_min_height_rate),
			x_direction_dx_max_height_rate(_x_direction_dx_max_height_rate),
			x_direction_dy_max_height_rate(_x_direction_dy_max_height_rate),
			centers_max_sin_theta(_centers_max_sin_theta)
		{

		}

		const RuneParam operator=(const RuneParam & runeParam)
		{
			this->hsv_h_min = runeParam.hsv_h_min;
			this->hsv_h_max = runeParam.hsv_h_max;
			this->hsv_s_min = runeParam.hsv_s_min;
			this->hsv_s_max = runeParam.hsv_s_max;
			this->hsv_v_min = runeParam.hsv_v_min;
			this->hsv_v_max = runeParam.hsv_v_max;

			this->contour_min_area = runeParam.contour_min_area;
			this->contour_boundingrect_min_aspect_ratio = runeParam.contour_boundingrect_min_aspect_ratio;
			this->contour_boundingrect_max_aspect_ratio = runeParam.contour_boundingrect_max_aspect_ratio;
			this->contour_boundingrect_min_area = runeParam.contour_boundingrect_min_area;
			this->contour_boundingrect_max_area = runeParam.contour_boundingrect_max_area;

			this->y_direction_dx_max_width_rate = runeParam.y_direction_dx_max_width_rate;
			this->y_direction_dy_max_height_rate = runeParam.y_direction_dy_max_height_rate;
			this->x_direction_dx_min_height_rate = runeParam.x_direction_dx_min_height_rate;
			this->x_direction_dx_max_height_rate = runeParam.x_direction_dx_max_height_rate;
			this->x_direction_dy_max_height_rate = runeParam.x_direction_dy_max_height_rate;
			this->centers_max_sin_theta = runeParam.centers_max_sin_theta;
			return *this;
		}
	

		int hsv_h_min = 0;					/*过滤时候h通道选择保留的最小值*/
		int hsv_h_max = 34;					/*过滤时候h通道选择保留的最大值*/
		int hsv_s_min = 43;					/*过滤时候s通道选择保留的最小值*/
		int hsv_s_max = 255;					/*过滤时候s通道选择保留的最大值*/
		int hsv_v_min = 150;					/*过滤时候v通道选择保留的最小值*/
		int hsv_v_max = 255;					/*过滤时候v通道选择保留的最大值*/

		int contour_min_area = 130;				/*轮廓内部的最小面积*/
		int contour_boundingrect_min_aspect_ratio = 0.25;	/*轮廓外包矩形的最小长宽比*/
		int contour_boundingrect_max_aspect_ratio = 4;		/*轮廓外包矩形的最大长宽比*/
		int contour_boundingrect_min_area = 200;		/*轮廓外包矩形的最小面积*/
		int contour_boundingrect_max_area = 3000;		/*轮廓外包矩形的最大面积*/

		float y_direction_dx_max_width_rate = 0.5;		/*y方向上找临近轮廓时，两个矩形相比较dx相差基准矩形高度的最大倍数*/
		float y_direction_dy_max_height_rate = 2.5;		/*y方向上找临近轮廓时，两个矩形相比较dy相差基准矩形宽度的最大倍数*/
		float x_direction_dx_min_height_rate = 1.5;		/*x方向上找临近轮廓时，两个矩形相比较dx相差基准矩形高度的最小倍数*/
		float x_direction_dx_max_height_rate = 3.5;		/*x方向上找临近轮廓时，两个矩形相比较dx相差基准矩形高度的最大倍数*/
		float x_direction_dy_max_height_rate = 1.0;		/*x方向上找临近轮廓时，两个矩形相比较dy相差基准矩形高度的最大倍数*/
		float centers_max_sin_theta = 0.30;			/*两个轮廓外包矩形中心点之间夹角的正弦值*/

	};
	
    class RuneDetector
	{
	public:
        //RuneDetector() {};
        //~RuneDetector() {};
	enum RuneFlag
	{
		RUNE_NO = 0,				//未识别出
		RUNE_PART = 1,				//给出了神符区域但是未全部检测出
		RUNE_FULL = 2,				//全部检测成功
		SHOOT_NO = 3,				//现在还不能打
            	SHOOT_WAIT = 4,				//已经给出了打击的坐标点，但还未发射,
		SHOOT_YES = 5,				//已经发射
	};

	/*
	*	@brief 初始化，载入参数
	*/
        void init();

        /*
        *	@brief 载入图片
        */
        void setMode(int runeMode, const RuneParam& runeParam);

	/*
	*	@brief 载入图片
	*/
	cv::Mat loadImg(cv::Mat mat);

	/*
	*	@brief 检测和识别
	*/
        int caculate();

	/*
	*	@brief 选择要击打的神符上的数字
	*/
	cv::Point2f chooseTarget();

        /*
        *	@brief reset parameters
        */
        void resetPara();

	bool isShoot(bool shootSituation)
	{
		isFly_ = shootSituation;
		if (isFly_ == true)
		{
			shootFlag_ = SHOOT_YES;
			return true;
		}
		return false;
	}

		/*
		*	@brief 给出flag
		*/
        int getFlag() const;

#ifdef DEBUG_GET_EVERYTHING
		/*
		*	@brief 获取神符的ROI区域
		*/
		const cv::Rect getRuneRoi() const;

		/*
		*	@brief 获取九宫格的中心点坐标
		*/
		const std::array<cv::Point2f, 9> getCenters() const;

		/*
		*	@brief 获取九宫格的9个数字
		*/
		const std::array<int, 9> getRuneNumbers() const;
		/*
		*	@brief 获取LED的5个数字
		*/
		const std::vector<int> getLedNumbers() const;



#endif // DEBUG_GET_EVERYTHING

	private:
        /*
        *	this blob contains the contours' characters found after findContours()
        */
		struct Blob
		{
			Blob() = default;
			Blob(const cv::Point2f& pcenter, const float pwidth,
				 const float pheight, const float parea)
			{
				center = pcenter;
				width = pwidth;
				height = pheight;
				area = parea;

			}
			cv::Point2f center;						/*轮廓的外包矩形的中心*/
			float width;							/*轮廓的外包矩形的宽*/
			float height;							/*轮廓的外包矩形的高*/
			float area;							/*轮廓的面积减去轮廓内部所有子轮廓的面积得到的面积*/
			cv::Point coor = {0, 0};					/*对应图谱点的位置*/
		};

        int runeMode_ = 0;								/*是大幅还是小幅*/
		RuneParam params_;							/*各项参数*/

		//TODO: double flags
		int runeFlag_;								/*找到神符里九宫格的情况*/
		int shootFlag_;								/*是否能打击的情况*/

		cv::Mat srcImg_;							/*原图*/
		cv::Mat workImg_;							/*程序运行时使用的图*/
		cv::Mat perspMat_;
		cv::Mat grayImg_;
		cv::Mat debugImg_;
		const std::string debugImgName = "debug info";

		std::vector<std::vector<cv::Point>> contours_;				/*findContours所找到的轮廓*/
		std::vector<cv::Vec4i> hierachy_;					/*findContours所找到的轮廓信息*/

		std::vector<Blob> blobs_;						/*初步筛选后保存留下轮廓的重要信息*/
		std::vector<cv::Rect2f> filteredRects_;					/*初步筛选后的轮廓外包矩形*/
		//std::vector<cv::Rect2f> final_Rects_;					/*经过第二次筛选后剩下的轮廓外包矩形*/
		std::vector<Blob> finalBlobs_;						/*经过第二次筛选后剩下的轮廓信息*/
		
		std::array<cv::Point2f, 9> runeCenters_;				/*神符的9个中心点*/
		std::array<int, 9> runeNumbers_;					/*神符的9个数字*/
		std::array<int, 9> oldruneNumbers_;					/*神符的9个数字*/
		std::array<std::pair<cv::Point2f, int>, 9> numbersNcenter_;		/*神符的9个数字和中心点*/
		cv::Rect rune_roi_;							/*神符的ROI区域*/

		cv::Rect ledRoi_;							/* 数码管搜索区域(相对srcImg) */		
		cv::Mat ledImg_;							/* 数码管图片 原图像直接裁剪下来的彩色图*/	
		
		cv::Mat ledProcessingImg_;						/*数码管部分用于进行一系列操作的一个变量*/
		std::vector<cv::Rect> ledRects_;					/* LED分割结果 */
		std::vector<int> runeLedNumbers_;					/*led的5个数字*/
		std::vector<int> oldruneLedNumbers_;					/*led上次的5个数字*/

		std::list<int> history_;						/*存储过去5次LED的结果*/
		int led_idx_ = 0;							/*记录现在打的是数码管的第几个数字(范围0~4)*/
		std::array<int, 10> vote;						/*记录投票结果*/
		cv::Point2f targetCoor_;						/*记录现在目标在九宫格图谱的坐标*/
		bool isFly_ = false;							/*记录子弹是否发射，发射了为true*/

		int debug_count_ = 1;
															
		/*
		*	@brief 判断是否是大幅
		*/
		int isGreatRune(int runeMode);


		/*
		*	@brief 从HSV图片里选取自己想要的颜色
		*	@param1 h通道的最小值
		*	@param2 h通道的最大值
		*	@param3 s通道的最小值
		*	@param4 s通道的最大值
		*	@param5 v通道的最小值
		*	@param6 v通道的最大值
		*	@information
		*		yellow channle of HSV
		*		H∈[26,34] S∈[43,255] V∈[46,255]
		*/
		void chooseColor(int hsv_h_min, int hsv_h_max,
			int hsv_s_min, int hsv_s_max,
			int hsv_v_min, int hsv_v_max);

		/*
		*	@brief 根据轮廓外包矩形的大小、面积以及二值化后轮廓内部黑点占比初步排除掉部分矩形
		*/
		void preScreen();

		/*
		*	@brief 采用深度优先搜索的思想对剩下的Blob进行过滤，
		*		对最后剩下的Blob构建图谱，通过对图谱中心的筛选，
		*		选出九宫格的中心，从而确定九宫格的位置
		*/
		void findRune();

		/*
		*	@brief 识别数码管的数字
		*/
        	const std::vector<int> findLedNumbers();

		/*
		*	@brief 	全局自适应阈值
		*			1.计算灰度直方图
		*			2.对灰度直方图进行6次多项式拟合
		*			3.通过差分的符号变化得到亮度最大的峰
		*			
		*	可以考虑对不同的情况用不同次数的多项式拟合，6次比较适合直方图有左中右三个峰的情况
		*/
		int findLEDThreshold(const cv::Mat & gray);

		/*
		*	@brief 数码数字识别，输入为数码数字图片，黑底白字
		*/
		int trans2RealLedNum(const cv::Mat& dframe_count);

		/*
		*	@brief 切割ledImg_，得到独立的数字区域
		*/
        	void getLedImg();

		
		/*
		*	@brief 寻找神符
		*/
		void  detect();

		/*
		*	@brief 分割和识别数字
		*/
		void recognize();

		/*将要打的数字记录到history_中，1-9记录成功，-1记录失败*/
		bool setRecord(int record);

		/*选择要击打的数字*/
		int chooseNumber();


	};

}
