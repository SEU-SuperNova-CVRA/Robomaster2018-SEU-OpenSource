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
#include "Rune.h"
#include "../General/numeric_rm.h"
#include "../General/opencv_extended.h"

//#define USE_DARKNET
#ifdef USE_DARKNET
#include "../Darknet/src/yolo_v2_class.hpp"
#endif // !USE_DARKNET

#include<stack>
#include<string>
#include<array>
#include<string>


using namespace std;
using namespace cv;


namespace rm
{
void showNumImgs(const std::array<cv::Mat, 9>& numImgs, const cv::Size size, const bool oneWindowForAll);

array<int, 9> getNumResult(std::vector<float*>& numOuts)
{
	array<int, 9> buffNums;
	buffNums.fill(0);

	typedef struct { int cell; int num; float val; } IdxNumVal;

	array<IdxNumVal, 81> idxNumVals;
	auto pixIter = numOuts.begin();
	auto idxNumValIter = idxNumVals.begin();
	for(int cell = 0; cell < 9; cell++)
	{

		for(int num = 1; num < 10; num++)
		{
			*idxNumValIter = IdxNumVal{cell, num, *(*pixIter + num - 1)};

			idxNumValIter++;
		}
		pixIter++;
	}
	sort(idxNumVals.begin(), idxNumVals.end(), [](const IdxNumVal& idxNumVal1, const IdxNumVal& idxNumVal2)
	{
		return idxNumVal1.val > idxNumVal2.val;
	});
	int done = 0;
	array<bool, 10> hasFound; hasFound.fill(false);
	for(int i = 0; i < 81; i++)
	{
		if(buffNums[idxNumVals[i].cell] == 0 && !hasFound[idxNumVals[i].num])
		{
			buffNums[idxNumVals[i].cell] = idxNumVals[i].num;
			hasFound[idxNumVals[i].num] = true;
			done++;
		}
		if(done == 9)break;
	}
	return buffNums;
}
void RuneDetector::init()
{
#ifdef USE_DARKNET
    Classifier& cla = Classifier::getInstance();
    cla.Init("/home/nvidia/Robomaster/Robomaster2018/Darknet/cfg/mnist_lenet.cfg", "/home/nvidia/Robomaster/Robomaster2018/Darknet/weights/mnist_lenet.weights", 9);
//	std::cout << "INit success !!!____ \n";
#endif // !USE_DARKNET
}

void RuneDetector::setMode(int runeMode, const RuneParam & runeParam/*RuneDetector::isGreatRune(int runeMode)*/)
{
    if(runeMode_ !=runeMode)
    {
        resetPara();
        //int judge = isGreatRune(runeMode);
        runeMode_ = runeMode;
        params_ = runeParam;
        if(runeMode_ == rm::MINI_RUNE)
        {
            //const RuneParam a;
            params_ = RuneParam(0, 0, 0, 0, 0, 0,
                                300, 0.5, 2, 1000, 10000,
                                0.5, 1.5, 0.5, 3.0, 1.0);
        }
        runeFlag_ = RUNE_NO;
        runeFlag_ = SHOOT_NO;
    }

}

cv::Mat RuneDetector::loadImg(cv::Mat mat)
{
	srcImg_ = mat;
	workImg_ = srcImg_.clone();
	debugImg_ = srcImg_.clone();
	return srcImg_;
}

cv::Point2f RuneDetector::chooseTarget()
{
	if (runeFlag_ == SHOOT_WAIT)
	{
		return targetCoor_;
	}
	int num = chooseNumber();
    //cout<<"choosed num is "<<num<<endl;
	if(num == -1)
	{
		return cv::Point2f(-1, -1);
	}
	else
	{
		for(auto i : numbersNcenter_)
		{
			if(i.second == num)
			{
				targetCoor_ = i.first;
				//runeFlag_ = SHOOT_WAIT;
				runeFlag_ = SHOOT_YES;
				return i.first;
			}
		}
	}
	//return cv::Point2f();
}

void RuneDetector::resetPara()
{
	led_idx_ = 0;
	oldruneNumbers_.fill(0);
	oldruneLedNumbers_.clear();
    //cout<<"reset led_idx and old data"<<endl;
}

int RuneDetector::caculate()
{
	detect();
    if(runeFlag_ != RUNE_NO && runeFlag_ != SHOOT_WAIT)
	{
		recognize();

#ifdef CASUAL_SET_NUMBER
        runeLedNumbers_ = std::vector<int>{6,6,6,6,6};
#else
        runeLedNumbers_ = findLedNumbers();
#endif

	}
    else if (runeFlag_ == SHOOT_WAIT)
	{
		float dist = 10000;
		cv::Point2f finalPoint= { -1,-1 };
		for (auto point : runeCenters_)
		{
			float cmpDist = std::abs(point.x - targetCoor_.x) + std::abs(point.y - targetCoor_.y);
			if (cmpDist < dist)
			{
				dist = cmpDist;
				finalPoint = point;
			}
		}
		if (finalPoint.x != -1 && finalPoint.y != -1)
		{
			targetCoor_ = finalPoint;
		}
	}
#ifdef DEBUG_FULL
	imshow(debugImgName, debugImg_);
    waitKey(1);

#endif // DEBUG_FULL

	return runeFlag_;
}

void RuneDetector::detect()
{
	blobs_.clear();
	filteredRects_.clear();
	finalBlobs_.clear();
	runeCenters_.fill({-1, -1});
	oldruneLedNumbers_ = runeLedNumbers_;
	oldruneNumbers_ = runeNumbers_;

	
	if (runeMode_ == rm::GREAT_RUNE)
	{
		cv::cvtColor(workImg_, workImg_, CV_BGR2HSV);
		chooseColor(params_.hsv_h_min, params_.hsv_h_max, params_.hsv_s_min, params_.hsv_s_max, params_.hsv_v_min, params_.hsv_v_max);
	}
	/*二值化，滤波处理*/
	if (runeMode_ == rm::MINI_RUNE)
	{
		cv::cvtColor(workImg_, workImg_, CV_BGR2GRAY);
	}
	cv::threshold(workImg_, workImg_, 100, 255, CV_THRESH_OTSU);
	if(runeMode_ == rm::MINI_RUNE)
	{
		cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		morphologyEx(workImg_, workImg_, cv::MORPH_OPEN, element);
		morphologyEx(workImg_, workImg_, cv::MORPH_CLOSE, element);
	}
	contours_.clear();
	hierachy_.clear();

#ifdef SHOW_PRETREAT
	cv::imshow("src", srcImg_);
	cv::imshow("threshold", workImg_);
	cv::waitKey(1);
#endif // SHOW_PRETREAT

	/*采用树结构找轮廓*/
	cv::findContours(workImg_, contours_, hierachy_, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

#ifdef SHOW_PRETREAT
	cv::Mat show_contours;
	cvex::showContours("showcontours", srcImg_, show_contours, contours_, cvex::BLUE);
#endif // SHOW_PRETREAT

	if (!contours_.empty())
	{
		preScreen();
	}

	findRune();

#ifdef DEBUG_FULL
    //show the finding result
	for(auto point : runeCenters_)
	{
		cvex::drawCrossing(debugImg_, debugImg_, point, cvex::RED);
	}
#endif // DEBUG_FULL

	if (runeFlag_ != RUNE_NO)
	{
		array<Point2f, 9> real9Centers;
		{
			int i = 0;
			for (int row = 0; row < 3; row++)
			{
				for (int col = 0; col < 3; col++)
				{
					real9Centers[i].y = (row + 1) * (32 + 12) + 32 / 2;
					real9Centers[i].x = col * (56 + 18) + 56 / 2;
					i++;
				}
			}
		}

		vector<Point2f> srcPoints;
		vector<Point2f> dstPoints;
		for (int i = 0; i < 9; i++)
		{
			if (runeCenters_[i].x > 0 && runeCenters_[i].y > 0)
			{
				srcPoints.push_back(runeCenters_[i]);
				dstPoints.push_back(real9Centers[i]);
			}
		}

		perspMat_ = cv::findHomography(srcPoints, dstPoints, noArray(), RANSAC);

		cvtColor(srcImg_, grayImg_, COLOR_BGR2GRAY);

		//预测没有别检测到的目标中心
		const Mat invPerspMat = perspMat_.inv();
		for (int i = 0; i < 9; i++)
		{
			if (runeCenters_[i].x <= 0 || runeCenters_[i].y <= 0)
			{
				Point2f realPt = real9Centers[i];
				const auto x = invPerspMat.at<double>(0, 0) * realPt.x + invPerspMat.at<double>(0, 1) * realPt.y + invPerspMat.at<double>(0, 2);
				const auto y = invPerspMat.at<double>(1, 0) * realPt.x + invPerspMat.at<double>(1, 1) * realPt.y + invPerspMat.at<double>(1, 2);
				const auto z = invPerspMat.at<double>(2, 0) * realPt.x + invPerspMat.at<double>(2, 1) * realPt.y + invPerspMat.at<double>(2, 2);
				runeCenters_[i].x = x / z;
				runeCenters_[i].y = y / z;

#ifdef DEBUG_FULL
				cvex::drawCrossing(debugImg_, debugImg_, runeCenters_[i], cvex::YELLOW);
#endif // DEBUG_FULL

			}
		}
	}

//set numbers for debug
#ifdef CASUAL_SET_NUMBER	
/*	
	if(debug_count_ == 1)
	{
		for(int i = 1; i <= 9; ++i)
		{
			runeNumbers_[i - 1] = i;
		}

	}
	else if(debug_count_ > 1 && debug_count_ % 10 == 0)
	{
		int t = runeNumbers_[0];
		for(int i = 0; i < 8; ++i)
		{
			runeNumbers_[i] = runeNumbers_[i + 1];

		}
		runeNumbers_[8] = t;
	}
	debug_count_++;
	for(int i = 1; i <= 9; ++i)
	{
		numbersNcenter_[i - 1].first = *(runeCenters_.begin() + i - 1);
		numbersNcenter_[i - 1].second = runeNumbers_[i - 1];
	}
*/
#endif // CASUAL_SET_NUMBER

}

void RuneDetector::recognize()
{
	Mat runeImg;
	cv::warpPerspective(grayImg_, runeImg, perspMat_, Size(56 * 3 + 18 * 2, 32 * 4 + 12 * 3));

	array<Mat, 9>numImgs;
	{
		int i = 0;
		for(int row = 0; row < 3; row++)
		{
			for(int col = 0; col < 3; col++)
			{
				Mat& img = numImgs[i] = runeImg(Rect(Point(col*(56 + 18) + 12, row*(32 + 12)+32+12), Size(32, 32)));
				resize(img, img, Size(28, 28));
				if(runeMode_ == MINI_RUNE)
				{
					threshold(numImgs[i], numImgs[i], 175, 255, cv::THRESH_OTSU);
				}
				cvtColor(img, img, COLOR_GRAY2BGR);
				i++;
				//imwrite("C:/Users/Binyan Hu/source/repos/Robomaster2018/Armor/Digits/" + to_string(getTickCount()) + ".jpg", img);				
			}
		}
		ledImg_ = runeImg(Rect({0,0}, Size(runeImg.cols, 32)));

		/*
		*	此处加入神经网络识别，
		*	传进神经网络的图为numImgs，
		*	并将识别出的数字放入runeNumbers_里，
		*	然后将这个数字对应的坐标和该数字放入numbersNcenter_里
		*/
	#ifdef USE_DARKNET
        //clock_t time1 = clock();
		Classifier& cla = Classifier::getInstance();
		std::vector<cv::Mat> tmp_ms(numImgs.begin(), numImgs.end());

        auto _v = cla.classifyAndReturnLayers(tmp_ms);
		runeNumbers_ = getNumResult(_v);

        //clock_t time2 = clock();
        //std::cout << std::endl << (float)(time2 - time1) / CLOCKS_PER_SEC << "s" << std::endl;

		//auto _v = cla.classify(tmp_ms);
		//std::copy(_v.begin(), _v.end(), runeNumbers_.begin());

		for(int _i = 0; _i < 9; _i++)
		{
			numbersNcenter_[_i] = std::make_pair(runeCenters_[_i], runeNumbers_[_i]);
		}
	#endif // !USE_DARKNET
	}

#ifdef DEBUG_NUM
    //show the remaining image after cutting and warpperspecting
	imshow("rune image", runeImg);
    //showNumImgs(numImgs, Size(28, 28), true);
	waitKey(1);
#endif // DEBUG_NUM

}

void RuneDetector::chooseColor(int hsv_h_min, int hsv_h_max, int hsv_s_min, int hsv_s_max, int hsv_v_min, int hsv_v_max)
{
	cv::Mat hsvImg;
	cv::cvtColor(srcImg_, hsvImg, CV_BGR2HSV);
	cv::inRange(hsvImg, cv::Scalar(hsv_h_min, hsv_s_min, hsv_v_min), cv::Scalar(hsv_h_max, hsv_s_max, hsv_v_max), workImg_);

#ifdef SHOW_PRETREAT
	cv::imshow("dealedImg", workImg_);
	cv::waitKey(1);
#endif // SHOW_PRETREAT

}

void RuneDetector::preScreen()
{
	cv::Mat mask;
	cv::Rect2f rect;
	for(int idx = 0; idx >= 0; idx = hierachy_[idx][0])/*TODO.....此处只是遍历了最外层轮廓，需要改为全部轮廓。*/
	{
		rect = cv::boundingRect(contours_[idx]);
		//根据长宽比和面积大小去除轮廓不正常的
		if(rect.width / rect.height > params_.contour_boundingrect_max_aspect_ratio ||
		   rect.width / rect.height < params_.contour_boundingrect_min_aspect_ratio ||
		   rect.area() < params_.contour_boundingrect_min_area ||
		   rect.area() > params_.contour_boundingrect_max_area)
			continue;
		//filteredRects_.push_back(rect);

        cv::Point2f center = (rect.tl() + rect.br()) / 2;
        float height = rect.height;
        float width = rect.width;
        float area = cv::contourArea(contours_[idx]);
        if(hierachy_[idx][2] != -1)								/*如果有子轮廓*/
        {
            int idxnext = hierachy_[idx][2];
            area -= cv::contourArea(contours_[idxnext]);			/*首先删除子轮廓的大小*/
            while(hierachy_[idxnext][0] != -1)					/*如果子轮廓有同级轮廓，*/
            {
                idxnext = hierachy_[idxnext][0];
                area -= cv::contourArea(contours_[idxnext]);		/*则该轮廓所占的大小为自己减去所有同级轮廓的大小*/
            }

        }
        if(area < params_.contour_min_area)
            continue;
        filteredRects_.push_back(rect);

#ifdef SHOW_FIRST_FILTER_PROCESS
        cv::Mat dst1 = srcImg_.clone();
        cv::rectangle(dst1, rect, cvex::RED, 2);
        cv::imshow("filtering", dst1);
        cv::waitKey(1);
#endif // SHOW_FIRST_FILTER_PROCESS

        blobs_.push_back(Blob{center, width, height, area});
    }

}

void RuneDetector::findRune()
{
	std::vector<Blob> dealing_blobs_;						/*处理中的blobs_*/
	cv::Mat showSearching;
	Blob baseBlob;
	runeFlag_ = RUNE_NO;
	while(blobs_.size() >= 7)
	{
		dealing_blobs_.push_back(blobs_.front());
		blobs_.erase(blobs_.begin());						/*处理过的Blob都将其从过滤Blob中删除*/
		int countRect = 0;									/*用于记放进每次循环放入final里的Blob个数*/
		while(!dealing_blobs_.empty())						/*只要还有待处理的Blob*/
		{
			baseBlob = dealing_blobs_.front();			/*取出第一个作为基准*/
			dealing_blobs_.erase(dealing_blobs_.begin());
			finalBlobs_.push_back(baseBlob);				/*放入最后的Blob中*/
			++countRect;
		#ifdef SHOW_TWICE_FILTER_PROCESS
			showSearching = srcImg_.clone();
			//cv::drawMarker(showSearching, baseBlob.center, cvex::BLUE, cv::MARKER_SQUARE, 30, 2);
			cv::circle(showSearching, baseBlob.center, 5, cvex::GREEN, 3);
			cv::imshow("showSearching", showSearching);
			cv::waitKey(0);
		#endif // SHOW_TWICE_FILTER_PROCESS

			/*遍历Blobs来找与基准Blob相近的Blob*/
			for(auto cmpBlobIdx = blobs_.begin(); cmpBlobIdx != blobs_.end();)
			{
				bool ischangeIdx = false;
				float dx = std::abs(cmpBlobIdx->center.x - baseBlob.center.x);
				float dy = std::abs(cmpBlobIdx->center.y - baseBlob.center.y);
				float dist = cvex::distance(cmpBlobIdx->center, baseBlob.center);
				//y方向找邻居。
				if(dx < baseBlob.width * params_.y_direction_dx_max_width_rate)
				{
					if(dy < baseBlob.height * params_.y_direction_dy_max_height_rate &&
					   dx / dist < params_.centers_max_sin_theta)
					{
						cmpBlobIdx->coor.x = baseBlob.coor.x;
						if(cmpBlobIdx->center.y > baseBlob.center.y)
						{
							cmpBlobIdx->coor.y = baseBlob.coor.y + 1;
						}
						else
						{
							cmpBlobIdx->coor.y = baseBlob.coor.y - 1;
						}
					#ifdef SHOW_TWICE_FILTER_PROCESS
						//cv::drawMarker(showSearching, cmpBlobIdx->center, cvex::BLUE, cv::MARKER_SQUARE, 30, 2);
						cv::circle(showSearching, cmpBlobIdx->center, 5, cvex::GREEN, 3);

						cv::imshow("showSearching", showSearching);
						cv::waitKey(1);
					#endif // SHOW_TWICE_FILTER_PROCESS

						//ydist = cvex::distanceManhattan(cmpBlobIdx->center, baseBlob.center);
						dealing_blobs_.push_back(*cmpBlobIdx);
						cmpBlobIdx = blobs_.erase(cmpBlobIdx);
						ischangeIdx = true;
					}
				}
				//x方向找邻居。
				else if(dy < baseBlob.height * params_.x_direction_dy_max_height_rate)
				{
					/*由于7,9这些数字的宽太小，不应该作为判据，故选择他们的高来作为判据*/
					if(dy / dist < params_.centers_max_sin_theta &&
					   rmnum::absApproxEqual(cmpBlobIdx->center.x, baseBlob.center.x, baseBlob.height * params_.x_direction_dx_min_height_rate, baseBlob.height * params_.x_direction_dx_max_height_rate) /*&&
																																														  isFindxRect == false*/)
					{
						cmpBlobIdx->coor.y = baseBlob.coor.y;
						if(cmpBlobIdx->center.x > baseBlob.center.x)
						{
							cmpBlobIdx->coor.x = baseBlob.coor.x + 1;
						}
						else
						{
							cmpBlobIdx->coor.x = baseBlob.coor.x - 1;
						}
					#ifdef SHOW_TWICE_FILTER_PROCESS
						//cv::drawMarker(showSearching, cmpBlobIdx->center, cvex::BLUE, cv::MARKER_SQUARE, 30, 2);
						cv::circle(showSearching, cmpBlobIdx->center, 5, cvex::GREEN, 3);
						cv::imshow("showSearching", showSearching);
						cv::waitKey(1);
					#endif // SHOW_TWICE_FILTER_PROCESS
						//xdist = cvex::distanceManhattan(cmpBlobIdx->center, baseBlob.center);
						dealing_blobs_.push_back(*cmpBlobIdx);
						cmpBlobIdx = blobs_.erase(cmpBlobIdx);
						ischangeIdx = true;
					}
				}
				if(!ischangeIdx) cmpBlobIdx++;
			}
		}
		if(countRect < 7)
		{
			finalBlobs_.clear();
			continue;
		}
		int xmax = finalBlobs_[0].coor.x;
		int xmin = finalBlobs_[0].coor.x;
		int ymax = finalBlobs_[0].coor.y;
		int ymin = finalBlobs_[0].coor.y;
		for(auto testblob : finalBlobs_)
		{
			xmax = std::max(xmax, testblob.coor.x);
			xmin = std::min(xmin, testblob.coor.x);
			ymax = std::max(ymax, testblob.coor.y);
			ymin = std::min(ymin, testblob.coor.y);
		}
		int col = xmax - xmin + 1;
		int row = ymax - ymin + 1;
		std::vector<Blob> centerBlob;
		if(col <= 2 || row <= 2)
		{
			finalBlobs_.clear();
			runeFlag_ = RUNE_NO;
			continue;
		}
		else
		{
			runeFlag_ = RUNE_PART;
			for(auto blob : finalBlobs_)
			{
				if(row > 3)
				{
					if(blob.coor.x == xmax ||
					   blob.coor.x == xmin ||
					   blob.coor.y <= ymax - 2 ||
					   blob.coor.y == ymin)
						continue;
					centerBlob.push_back(blob);

				}
				else
				{
					if(blob.coor.x == xmax ||
					   blob.coor.x == xmin ||
					   blob.coor.y == ymax ||
					   blob.coor.y == ymin)
						continue;
					centerBlob.push_back(blob);

				}
			}
			if(centerBlob.size() == 0)continue;
			Blob realCenter = centerBlob.front();
			if(centerBlob.size() != 1)
			{
				int cmpCount = 0;
				for(auto center : centerBlob)
				{
					int nearCount = 0;
					for(auto blob : finalBlobs_)
					{
						/*看周围有几个blob*/
						if(std::abs(blob.coor.x - center.coor.x) <= 1 &&
						   std::abs(blob.coor.y - center.coor.y) <= 1)
						{
							++nearCount;
						}
					}
					if(nearCount > cmpCount)
					{
						realCenter = center;
						cmpCount = nearCount;
					}
				}
			}
			float x1_roi = 0;
			float y1_roi = 0;
			float x2_roi = 0;
			float y2_roi = 0;
			float x8_roi = 0;
			float y8_roi = 0;
			for(auto blob : finalBlobs_)
			{
				if(blob.coor.y == realCenter.coor.y - 1)
				{
					if(blob.coor.x == realCenter.coor.x - 1)
					{
						runeCenters_[0] = blob.center;
						x1_roi = blob.center.x;
						y1_roi = blob.center.y;

					}
					else if(blob.coor.x == realCenter.coor.x)
					{
						runeCenters_[1] = blob.center;
						x2_roi = blob.center.x;
					}
					else if(blob.coor.x == realCenter.coor.x + 1)
					{
						runeCenters_[2] = blob.center;
					}
				}
				else if(blob.coor.y == realCenter.coor.y)
				{
					if(blob.coor.x == realCenter.coor.x - 1)
					{
						runeCenters_[3] = blob.center;
						y2_roi = blob.center.y;
					}
					else if(blob.coor.x == realCenter.coor.x)
					{
						runeCenters_[4] = blob.center;
					}
					else if(blob.coor.x == realCenter.coor.x + 1)
					{
						runeCenters_[5] = blob.center;
					}
				}
				else if(blob.coor.y == realCenter.coor.y + 1)
				{
					if(blob.coor.x == realCenter.coor.x - 1)
					{
						runeCenters_[6] = blob.center;
					}
					else if(blob.coor.x == realCenter.coor.x)
					{
						runeCenters_[7] = blob.center;
					}
					else if(blob.coor.x == realCenter.coor.x + 1)
					{
						runeCenters_[8] = blob.center;
						x8_roi = blob.center.x;
						y8_roi = blob.center.y;
					}
				}
			}
			runeFlag_ = RUNE_FULL;
			float x_roi = x1_roi - (x2_roi - x1_roi) / 2;
			float y_roi = y1_roi - (y2_roi - y1_roi) / 2;
			float xx_roi = x8_roi + (x2_roi - x1_roi) / 2;
			float yy_roi = y8_roi + (y2_roi - y1_roi) / 2;
			rune_roi_ = cv::Rect(cv::Point2f(x_roi, y_roi), cv::Point2f(xx_roi, yy_roi));
			for(auto i : runeCenters_)
			{
				if(i.x == -1 || i.y == -1)
					runeFlag_ = RUNE_PART;
			}
			break;/*已经能确定神符的位置，终止查找*/
		}
	}
	std::array<cv::Point2f, 9> negative;
	negative.fill({-1, -1});
	if(runeCenters_ == negative)
	{
		runeFlag_ = RUNE_NO;
	}
}

const std::vector<int> RuneDetector::findLedNumbers()
{
    getLedImg();
#ifdef DEBUG_NUM
        cv:imshow("led",ledProcessingImg_);
        cv::waitKey(1);
#endif
	std::vector<int> dframe_counts;
	cv::Mat bin;
	if(ledRects_.size() != 5)
	{
		ledRects_.clear();
		return std::vector<int>(5, -1);
	}
	for(auto i = ledRects_.begin(); i != ledRects_.end(); ++i)
	{
		ledProcessingImg_(*i).copyTo(bin);
		//threshold(ledImg_(*i), bin, findLEDThreshold(ledImg_(*i)), 255, CV_THRESH_BINARY);
		dframe_counts.push_back(trans2RealLedNum(bin));
	}
	ledRects_.clear();
	return dframe_counts;

}

int RuneDetector::findLEDThreshold(const cv::Mat & gray)
{
	//calc hist
	cv::Mat mhist;
	cv::calcHist(std::vector<cv::Mat>({gray}), std::vector<int>({0}), cv::Mat(), mhist, std::vector<int>({256}), std::vector<float>({0,255}));
	//std
	std::vector<float> x;
	for(int i = 0; i < 256; ++i)
	{
		x.push_back((i - 127.5) / 74.045);
	}
	//polyfit & calc zero point of diff
	auto f0 = rmnum::polyval(rmnum::polyfit(x, std::vector<float>(mhist.begin<float>(), mhist.end<float>()), 6), x);
	int threshold = 200;
	auto ptr0 = f0.rbegin();
	auto ptr1 = f0.rbegin() + 1;
	double diff = *ptr0++ - *ptr1++;
	for(; ptr1 != f0.rend();)
	{
		double lastDiff = diff;
		diff = *ptr0++ - *ptr1++;
		if(diff*lastDiff <= 0 && lastDiff > 0)
		{
			threshold = 255 - (ptr0 - f0.rbegin());
			break;
		}
	}
	return threshold;
}

int RuneDetector::trans2RealLedNum(const cv::Mat & dframe_count)
{
	if(float(dframe_count.rows) / float(dframe_count.cols) > 2.0)
	{
		return 1;
	}
	cv::Mat leftD, leftM;
	leftM = dframe_count(cv::Range(dframe_count.rows / 3, 2 * dframe_count.rows / 3), cv::Range(0, dframe_count.cols / 3)); //左边中间
	leftD = dframe_count(cv::Range(2 * dframe_count.rows / 3, dframe_count.rows), cv::Range(0, dframe_count.cols / 3)); //左下角
	auto end = leftD.end<unsigned char>();
	int tmp = 0;
	for(auto i = leftD.begin<unsigned char>(); i != end; ++i)
	{
		tmp += *i;
	}
	if(tmp <= 255) //如果左下角没有，则可能是4 7
	{
		end = leftM.end<unsigned char>();
		for(auto i = leftM.begin<unsigned char>(); i != end; ++i)
		{
			tmp += *i;
		}
		if(tmp > 255) //左下角没有，左边中间有则是 4
		{
			return 4;
		}
		else
		{
			return 7;
		}

	}
	cv::Mat row0, row1, col0;
	row0 = dframe_count.row(dframe_count.rows / 4 + 1);
	row1 = dframe_count.row(3 * dframe_count.rows / 4);
	col0 = dframe_count.col(dframe_count.cols / 2);
	unsigned char seg = 0;
	tmp = 0;
	int count = 0;
	end = col0.end<unsigned char>();
	for(auto i = col0.begin<unsigned char>(); i != end; ++i, ++count) //检查三段横着的数码管是否亮
	{
		tmp += *i;
		if(count == dframe_count.rows / 4 + 1)
		{
			if(tmp == 255)
			{
				cv::Mat colL, colR;
				colL = dframe_count.col(dframe_count.cols / 2 - 1);
				colR = dframe_count.col(dframe_count.cols / 2 + 1);
				auto j = colR.begin<unsigned char>();
				for(int k = 0; k < count; ++k)
				{
					tmp += *j++;
				}
				j = colL.begin<unsigned char>();
				for(int k = 0; k < count; ++k)
				{
					tmp += *j++;
				}
			}
			if(tmp > 255)
			{
				seg |= 1;
			}
			tmp = 0;
		}
		else if(count == 3 * dframe_count.rows / 4)
		{
			if(tmp == 255)
			{
				cv::Mat colL, colR;
				colL = dframe_count.col(dframe_count.cols / 2 - 1);
				colR = dframe_count.col(dframe_count.cols / 2 + 1);
				auto j = colR.begin<unsigned char>() + dframe_count.rows / 4 + 1;
				for(int k = 0; k < dframe_count.rows / 2; ++k)
				{
					tmp += *j++;
				}
				j = colL.begin<unsigned char>() + dframe_count.rows / 4 + 1;
				for(int k = 0; k < dframe_count.rows / 2; ++k)
				{
					tmp += *j++;
				}
			}
			if(tmp > 255)
			{
				seg |= 1 << 6;
			}
			tmp = 0;
		}
	}
	if(tmp == 255)
	{
		cv::Mat colL, colR;
		colL = dframe_count.col(dframe_count.cols / 2 - 1);
		colR = dframe_count.col(dframe_count.cols / 2 + 1);
		end = colR.end<unsigned char>();
		for(auto j = colR.begin<unsigned char>() + 3 * dframe_count.rows / 4; j != end; ++j)
		{
			tmp += *j;
		}
		end = colL.end<unsigned char>();
		for(auto j = colL.begin<unsigned char>() + 3 * dframe_count.rows / 4; j != end; ++j)
		{
			tmp += *j;
		}
	}
	if(tmp > 255)
	{
		seg |= 1 << 3;
	}
	tmp = 0;
	count = 0;
	end = row0.end<unsigned char>();
	for(auto i = row0.begin<unsigned char>(); i != end; ++i, ++count) //检查上面两段竖着的数码管是否亮
	{
		tmp += *i;
		if(count == dframe_count.cols / 2)
		{
			if(tmp == 255)
			{
				cv::Mat rowD = dframe_count.row(dframe_count.rows / 4 + 2);
				auto j = rowD.begin<unsigned char>();
				for(int k = 0; k < count; ++k)
				{
					tmp += *j++;
				}
			}
			if(tmp > 255)
			{
				seg |= 1 << 5;
			}
			tmp = 0;
		}
	}
	if(tmp == 255)
	{
		cv::Mat rowD = dframe_count.row(dframe_count.rows / 4 + 2);
		end = rowD.end<unsigned char>();
		for(auto j = rowD.begin<unsigned char>() + dframe_count.cols / 2; j != end; ++j)
		{
			tmp += *j;
		}
	}
	if(tmp > 255)
	{
		seg |= 1 << 1;
	}
	tmp = 0;
	count = 0;
	end = row1.end<unsigned char>();
	for(auto i = row1.begin<unsigned char>(); i != end; ++i, ++count)  //检查下面两端竖着的数码管是否亮
	{
		tmp += *i;
		if(count == dframe_count.cols / 2)
		{
			if(tmp == 255)
			{
				cv::Mat rowU = dframe_count.row(3 * dframe_count.rows / 4 - 1);
				auto j = rowU.begin<unsigned char>();
				for(int k = 0; k < count; ++k)
				{
					tmp += *j++;
				}
			}
			if(tmp > 255)
			{
				seg |= 1 << 4;
			}
			tmp = 0;
		}
	}
	if(tmp == 255)
	{
		cv::Mat rowU = dframe_count.row(3 * dframe_count.rows / 4 - 1);
		end = rowU.end<unsigned char>();
		for(auto j = rowU.begin<unsigned char>() + dframe_count.cols / 2; j != end; ++j)
		{
			tmp += *j;
		}
	}
	if(tmp > 255)
	{
		seg |= 1 << 2;
	}
	//std::cout << int(seg) << " ";
	switch(seg)
	{
	case 63:
	return 0;
	case 6:
	return 1;
	case 91:
	return 2;
	case 79:
	return 3;
	case 102:
	return 4;
	case 109:
	return 5;
	case 125:
	return 6;
	case 7:
	return 7;
	case 15:
	return 7;
	case 127:
	return 8;
	case 111:
	return 9;
	default:
	break;
	}
	return -1;

}

void RuneDetector::getLedImg()
{
    //First step: Threshold
    cv::threshold(ledImg_, ledProcessingImg_, 100, 255, cv::THRESH_OTSU);

    //Second step: WarpAffine
    std::vector<cv::Point> points;
    for(int i = 0; i < ledProcessingImg_.rows; ++i)
    {
        auto p = ledProcessingImg_.ptr<unsigned char>(i);
        for(int j = 0; j < ledProcessingImg_.cols; ++j)
        {
            if(*p++)
            {
                points.push_back(cv::Point(j, i));
            }
        }
    }
    if(points.size() == 0)
        return;
    cv::RotatedRect roi = cv::minAreaRect(points);//框出5个数
    cv::Point2f apexs[4];
    cv::Point2f apexs_sort[4];
    roi.points(apexs);
    float x_center = 0, y_center = 0;
    x_center = (apexs[0].x + apexs[1].x + apexs[2].x + apexs[3].x) / 4;
    y_center = (apexs[0].y + apexs[1].y + apexs[2].y + apexs[3].y) / 4;
    for(int i = 0; i < 4; ++i)
    {
        if(apexs[i].x < x_center && apexs[i].y < y_center)
        {
            apexs_sort[1] = apexs[i];
        }
        else if(apexs[i].x > x_center && apexs[i].y < y_center)
        {
            apexs_sort[2] = apexs[i];
        }
        else if(apexs[i].x < x_center && apexs[i].y > y_center)
        {
            apexs_sort[0] = apexs[i];
        }
        else
        {
            apexs_sort[3] = apexs[i];
        }
    }
    for(int i = 0; i < 4; ++i)
    {
        apexs[i] = apexs_sort[i];
    }
    float width = std::max(roi.size.width, roi.size.height);
    float height = std::min(roi.size.width, roi.size.height);

    cv::Point2f s[3] = {apexs[0],apexs[1],apexs[2]};
    cv::Point2f d[3] = {cv::Point2f(width / 20,height * 19 / 20), cv::Point2f(width / 20, height / 20), cv::Point2f(width * 19 / 20, height / 20)};
    cv::warpAffine(ledProcessingImg_, ledProcessingImg_, cv::getAffineTransform(s, d), cv::Size(int(width), int(height)));//仿射变换

    //Third step: divide led into 5 numbers
    cv::Mat mLEDframe_countber;
    std::vector<int> colheight(ledProcessingImg_.cols, 0);
    int value;
    for(int i = 0; i < ledProcessingImg_.rows; i++)//i表示纵坐标
    {
        for(int j = 0; j < ledProcessingImg_.cols; j++)//j表示横坐标
        {
            value = ledProcessingImg_.at<uchar>(i, j);//第一个为纵坐标 第二个为横坐标
            if(value == 255)
            {
                colheight[j]++;//统计每列白色像素点
            }
        }
    }
    int start = 0, count = 0, end = 0, length = 0;
    for(int k = 0; k < ledProcessingImg_.cols; ++k)
    {
        if(colheight[k] == 0)
        {
            cv::Rect LED_Rect;
            LED_Rect = cv::Rect(start, 0, length + 1, ledProcessingImg_.size().height);
            if(count >= 3)
            {
                if(LED_Rect.tl().x + LED_Rect.width > ledImg_.size().width || LED_Rect.tl().y +LED_Rect.height > ledImg_.size().height) return;
                mLEDframe_countber = ledImg_(LED_Rect);

                //cv::imwrite("D:\\visual studio 2015 gitgarage\\Robomaster\\Buff Recognition\\test\\" + std::to_string(frame_count++) + ".jpg", mLEDframe_countber);
                ledRects_.push_back(LED_Rect);
            }
            count = 0;
            start = k;
        }
        else
        {
            ++count;
            end = k;
            length = end - start;
            if(k == 0)//第一列就有亮点
                start = 0;
            if(k == ledProcessingImg_.cols - 1 && count > 3 && ledRects_.size() <= 5)//最后一列还有亮点
            {
                cv::Rect LED_Rect;
                LED_Rect = cv::Rect(start, 0, length + 1, ledProcessingImg_.size().height);
                if(LED_Rect.width > ledImg_.size().width || LED_Rect.height > ledImg_.size().height) return;
                mLEDframe_countber = ledImg_(LED_Rect);
                ledRects_.push_back(LED_Rect);
            }
        }
    }

}

bool RuneDetector::setRecord(int record)
{
	if(record < 0) return false;
	/*只保留过去5次的结果*/
	if(history_.size() < 6)
	{
		history_.push_back(record);
		return true;
	}
	history_.push_back(record);
	history_.pop_front();
	return true;
}

int RuneDetector::chooseNumber()
{
	/*
	*	打第一个数字的可能：
	*		1、一开始，history_还没有记录
	*		2、数码管数字不变，神符数字不变（未打）
	*		3、数码管数字变化，神符数字变化（打错）
	*/

	if(history_.empty()
	   || (runeLedNumbers_ != oldruneLedNumbers_ && runeNumbers_ != oldruneNumbers_)
	   /*|| (runeLedNumbers_ == oldruneLedNumbers_ && runeNumbers_ == oldruneNumbers_)*/)
	{
        //cout<<"old: "<<oldruneLedNumbers_[0]<<" "<<oldruneLedNumbers_[1]<<endl;
		setRecord(runeLedNumbers_[0]);
		led_idx_ = 0;
	}
	/*
	*	打下一个数字的条件：
	*		只有当数码管数字不变，并且神符数字变化（击打上一个数字成功）
	*/
	else if(runeLedNumbers_ == oldruneLedNumbers_ && runeNumbers_ != oldruneNumbers_)
	{
        if(led_idx_<4)
        {
            setRecord(runeLedNumbers_[++led_idx_]);
        }
        else
        {
            resetPara();//finish
        }
	}
	/*
	*	其他情况为还在当前数字
	*/
    else
	{
		setRecord(runeLedNumbers_[led_idx_]);
	}
    //cout<<"now ledidx is "<<led_idx_<<endl;
	/*
	*	投票
	*/
	vote.fill(0);
	if(history_.size() < 3)
	{
        runeFlag_ = SHOOT_NO;
		return -1;
	}
	else
	{
		for(int i : history_)
		{
			vote[i]++;
		}
		for(int j = 1; j <= 9; ++j)
		{
			/*必须这个数字达到3票以上，并且它是最后放进来的数字*/
			if(vote[j] >= 3 && history_.back() == j)
			{
                //runeFlag_ = SHOOT_WAIT;
                runeFlag_ = SHOOT_YES;
				return j;
			}
			if(j == 9 && (history_.back() != j || vote[9] < 3))
			{
                runeFlag_ = SHOOT_NO;
				return -1;
			}
		}
	}
}



const cv::Rect RuneDetector::getRuneRoi() const
{
	return rune_roi_;
}

int RuneDetector::getFlag() const
{
	return runeFlag_;
}

const std::array<cv::Point2f, 9> RuneDetector::getCenters() const
{
	return runeCenters_;
}

const std::array<int, 9> RuneDetector::getRuneNumbers() const
{
	return runeNumbers_;
}

const std::vector<int> RuneDetector::getLedNumbers() const
{
	return runeLedNumbers_;
}






void showNumImgs(const std::array<cv::Mat, 9>& numImgs, const cv::Size size, const bool oneWindowForAll)
{
	const int& width = size.width;
	const int& height = size.height;
	Mat mergeImg(Size(width * 3, height * 3), numImgs[0].type());
	Mat tmpImg;
	for(int i = 0; i < 9; i++)
	{
		if(!numImgs[i].empty())
		{
			resize(numImgs[i], tmpImg, size);
		}
		else
		{
			tmpImg = Mat::zeros(size, CV_8UC1);
		}

		if(oneWindowForAll)
		{
			tmpImg.copyTo(mergeImg(Rect(i % 3 * width, i / 3 * height, width, height)));
		}
		else
		{
			cv::namedWindow(std::to_string(i), CV_WINDOW_NORMAL);
			imshow(std::to_string(i), tmpImg);
		}
	}
	if(oneWindowForAll)
	{
		imshow("Numbers", mergeImg);
	}
}

}
