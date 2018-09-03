/*********************************************************
*描述：
*	尝试通过九宫格旁边的10个积分块来定位九宫格的位置
*最后结果：
*	不可行！暂时想不到解决方案。
*原因：
*	当获得的图片斜到一定程度时候，经二值化后会导致5个积分块
*	的一条边与旁边的墙连在一起，使得框不出该旋转矩形，形成漏检,
*	再倾斜一些的时候会导致一侧的所有积分块都检测不出。
*作者：
*	施殊, <213162637@seu.edu.cn>
*创建日期:
*	2018/4/21
***********************************************************/

#include "BuffDetector.h"
#include "../General/opencv_extended.h"
#include "../General/numeric_rm.h"

int count = 0;

//#define SHOW_THRESHOLD
//#define SHOW_CONTOURS
//#define SHOW_PROCESS_FOUND_RECT
//#define SHOW_FOUND_RECTS
#define SHOW_LAST_RECTS
//#define SHOW_ROTATEDRECT_BUFFROI

cv::Mat srcBuff;
cv::RotatedRect avrRect;
bool located_half = false;//记录是否有一边5个矩形找齐


enum locatedSituation
{
	LOCATED_NO = 0,//两边的5个矩形都没找满
	LOCATED_PART = 1,//两边的5个矩形有一边找满了，还有一边没找满
	LOCATED_FULL = 2,//10个矩形都找满了
};

int located_situation = LOCATED_NO;

////判断该容器的旋转矩形中宽大于高的数量多，还是高大于宽的数量多
//double Widthnum_MoreThan_Heightnum(std::vector<cv::RotatedRect> RotatedRects)
//{
//	double agree = 0;
//	for (int i = 0; i < RotatedRects.size(); ++i)
//	{
//		if (RotatedRects[i].size.width > RotatedRects[i].size.height)
//			agree++;
//		else if (RotatedRects[i].size.width < RotatedRects[i].size.height)
//			agree--;
//	}
//	return agree;
//}


cv::RotatedRect& adjustRec(cv::RotatedRect& rec)
{
	using std::swap;
	cv::Size2f& size = rec.size;
	float& angle = rec.angle;
	if (size.width < size.height)
	{
		swap(size.width, size.height);
		angle += 90.0;
	}
	while (angle >= 90.0) angle -= 180.0;
	while (angle < -90.0) angle += 180.0;
	if (size.height >= 0.5*size.width)
	{
		if (-90.0 <= angle && angle < -45.0)
		{
			swap(size.width, size.height);
			angle += 90.0;
		}
		else if (45.0 < angle && angle <= 90.0)
		{
			swap(size.width, size.height);
			angle -= 90.0;
		}
	}
	return rec;
}

//可以用groupRectangles尝试下。TODO.....
void calcAvrRect(std::vector<cv::RotatedRect> final_rects)
{
	float avrAngle = 0.0;
	float avrWidth = 0.0;
	float avrHeight = 0.0;
	float avrArea = 0.0;
	cv::Point2f avrCenter = (0, 0);
	for (auto rect : final_rects)
	{
		avrAngle += rect.angle;
		avrWidth += rect.size.width;
		avrHeight += rect.size.height;
		avrArea += rect.size.area();
		avrCenter += rect.center;
	}
	avrRect.angle = avrAngle / final_rects.size();
	avrRect.size.width = avrWidth / final_rects.size();
	avrRect.size.height = avrHeight / final_rects.size();
	avrRect.center = cv::Point(avrCenter.x / final_rects.size(), avrCenter.y / final_rects.size());
}

void buff_FindRoi(cv::Mat gray)
{

	cv::Mat show_contours = gray.clone();
	cv::threshold(gray, gray, 100, 255, CV_THRESH_OTSU);
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
	//cv::morphologyEx(gray, gray, cv::MORPH_OPEN, element);
	//cv::morphologyEx(gray, gray, cv::MORPH_ERODE, element);

#ifdef SHOW_THRESHOLD
		cv::imshow("threshold", gray);
		cv::waitKey(1);
#endif // SHOW_THRESHOLD
	//std::cout << count++ << std::endl;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::RotatedRect> RotatedRects;
	cv::findContours(gray, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
	//利用之前已找着的矩形的平均矩形来筛选新一轮的轮廓
	for (int i = 0; i < contours.size(); ++i)
	{
		cv::RotatedRect rect = adjustRec(cv::minAreaRect(contours[i]));
		if (located_situation == LOCATED_FULL || located_situation == LOCATED_PART)
		{
			if (rmnum::absApproxEqual(rect.angle, avrRect.angle, 15) //角度相差一定范围之内
				//&& rmnum::relApproxEqual(rect.size.area(), avrRect.size.area(), 0.5) //大小相对误差一定范围内
				&& rmnum::relApproxEqual(rect.size.width, avrRect.size.width, 0.4)
				&& rmnum::relApproxEqual(rect.size.height, avrRect.size.height, 0.4))
				//&& rmnum::relApproxEqual(rect.size.width / rect.size.height, avrRect.size.width / avrRect.size.height, 0.5)) //长宽比的相对误差在一定范围之内
			{
				RotatedRects.push_back(rect);
			}
		}
		else
		{
			if (rect.size.area() < 500 || rect.size.area() > 10000
				|| rect.size.width / rect.size.height < 0.25 
				|| rect.size.width / rect.size.height > 4)
			{
				continue;
			}
			RotatedRects.push_back(rect);
		}
	}


	std::vector<cv::RotatedRect> RotaRects_choose_nums(RotatedRects);

#ifdef SHOW_CONTOURS
	cvex::showRectangles("show_contours", RotatedRects, srcBuff, show_contours, cvex::GREEN, 1);
#endif // SHOW_CONTOURS

	located_situation = LOCATED_NO;
	//找到旁边10个矩形
	std::vector<cv::RotatedRect> found_rects;
	std::vector<cv::RotatedRect> final_rects;

	//从第一个往后面找，如果只剩下1个，就直接舍去了
	while (RotatedRects.size() > 1)
	{
		//已经找完10个旁边的矩形
		/*if (found_rects.size() == 10 )
			break;*/
			//选出第一个矩形作为基矩形
		cv::RotatedRect based_rect = adjustRec(*(RotatedRects.begin()));
		RotatedRects.erase(RotatedRects.begin());
		found_rects.push_back(based_rect);
		for (int i = 0; i < found_rects.size() - 1; ++i)
		{
			if (found_rects[i].center == based_rect.center)
				found_rects.pop_back();
		}
		int found_rectCount = 1;
#ifdef SHOW_PROCESS_FOUND_RECT
		cv::Mat show_found_rect = srcBuff.clone();
		cvex::showRectangle("found_rect", based_rect, srcBuff, show_found_rect, cvex::GREEN, 1);
#endif // SHOW_PROCESS_FOUND_RECT
		//将初步筛选的所有矩形都与其进行相关参数的比较

		for (auto compared_rectIdx = RotatedRects.begin() ; compared_rectIdx != RotatedRects.end() ; )
		{

			cv::RotatedRect compared_rect = adjustRec(*compared_rectIdx);
#ifdef SHOW_PROCESS_FOUND_RECT
			cv::Mat show_finding_rect = srcBuff.clone();
			cvex::showRectangle("show_finding_rect", *compared_rectIdx, srcBuff, show_finding_rect, cvex::RED, 100);
#endif // SHOW_PROCESS_FOUND_RECT
			//找到一个与自身最近的矩形
			if (rmnum::absApproxEqual(based_rect.angle, compared_rect.angle, 10) //角度相差一定范围之内
				//&& rmnum::relApproxEqual(based_rect.size.area(), compared_rect.size.area(), 0.35) //大小相对误差0.3以内
				&& rmnum::relApproxEqual(based_rect.size.width / based_rect.size.height, compared_rect.size.width / compared_rect.size.height, 0.3) //长宽比的相对误差在0.2之内				
				&& rmnum::absApproxEqual(based_rect.center.x, compared_rect.center.x, based_rect.size.width) //中心点横坐标相差在半个宽度内
				&& rmnum::absApproxEqual(based_rect.center.y, compared_rect.center.y, based_rect.size.height, based_rect.size.height * 3))
			{
				found_rects.push_back(compared_rect);
				++found_rectCount;
				based_rect = compared_rect;
				compared_rectIdx = RotatedRects.erase(compared_rectIdx);

#ifdef SHOW_PROCESS_FOUND_RECT
				cvex::showRectangle("found_rect", *compared_rectIdx, srcBuff, show_found_rect, cvex::BLUE, 1);
#endif // SHOW_PROCESS_FOUND_RECT
			}
			//没找到最近的且没找齐5个，但是找到了一个距离有点远但长得相似的
			else if (rmnum::absApproxEqual(based_rect.angle, compared_rect.angle, 10) //角度相差一定范围之内
				&& rmnum::relApproxEqual(based_rect.size.width / based_rect.size.height, compared_rect.size.width / compared_rect.size.height, 0.3) //长宽比的相对误差在0.2之内				
				&& rmnum::absApproxEqual(based_rect.center.x, compared_rect.center.x, based_rect.size.width) //中心点横坐标相差在一个宽度内
				&& rmnum::absApproxEqual(based_rect.center.y, compared_rect.center.y, based_rect.size.height, based_rect.size.height * 9)
				&& found_rectCount < 5)
			{
				found_rects.push_back(compared_rect);
				++found_rectCount;
				based_rect = compared_rect;
				compared_rectIdx = RotatedRects.erase(compared_rectIdx);
			}
			else
				compared_rectIdx++;
		}
		if (found_rectCount < 3)
		{
			found_rects.erase(found_rects.end() - found_rectCount, found_rects.end());
		}
		else if (found_rectCount > 5)
		{
			std::vector<double> dist;
			for (auto i = found_rects.end() - found_rectCount; i != found_rects.end() - 1; ++i)
			{
				double distance = cvex::distance((*i).center, (*(i + 1)).center);
				dist.push_back(distance);
			}

		}
		if (found_rectCount == 5)
		{
			//如果中间缺一个，下面找到一个怎么办？？TODO....
			located_half = true;
			for (auto i = found_rects.end() - 5; i != found_rects.end(); ++i)
			{
				final_rects.push_back(*i);
			}
			found_rects.erase(found_rects.end() - found_rectCount, found_rects.end());
		}
	}
#ifdef SHOW_FOUND_RECTS
	cv::Mat show_found_rects;
	cvex::showRectangles("show_found_rects", found_rects, srcBuff, show_found_rects, cvex::GREEN, 1);

#endif // SHOW_FOUND_RECTS

	if (located_half == false)
	{
		located_situation = LOCATED_NO;
	}
	else
	{
		//计算出最后找到的矩形的平均尺寸
		calcAvrRect(final_rects);
		if (final_rects.size() == 10)
		{
			located_situation = LOCATED_FULL;
		}
		else
		{
			located_situation = LOCATED_PART;
			//有一边的5个已经找到，另一边由于某种原因找不全，则将相似的矩形放进去，能找几个算几个
			if (found_rects.size() < 10)
			{
				for (auto i : found_rects)
				{
					if (/*rmnum::relApproxEqual(i.size.area(), avrRect.size.area(), 0.35) //大小基本相同
						&&*/ rmnum::relApproxEqual(i.size.width / i.size.height, avrRect.size.width / avrRect.size.height, 0.5) //长宽比基本相同			
						/*&& rmnum::absApproxEqual(i.center.y, avrRect.center.y, avrRect.size.height * 6)*/)
					{
						final_rects.push_back(i);
					}
				}
			}
			//找到了3列都有5个相似的矩形，则比较每一组的平均大小，取较接近的两组作为最后的矩形
			else
			{

			}
			//如果最后的10个矩形里长宽比大小差别太明显，则继续在found_rects里找可能的5个
			/*if (!rmnum::relApproxEqual((*final_rects.begin()).size.width / (*final_rects.begin()).size.height,
				(*(final_rects.begin() + 6)).size.width / (*(final_rects.begin() + 6)).size.height, 0.5))
			{
				for (auto i = found_rects.begin(); i != found_rects.end();)
				{
				}
			}*/
			cv::Mat avr;
			cvex::showRectangle("avrRect", avrRect, srcBuff, avr, cvex::RED, 1);
			calcAvrRect(final_rects);
		}
		//avrRect.size.area() = avrArea / found_rects.size();
	}
#ifdef SHOW_LAST_RECTS
	cv::Mat show_final_rects;
	cvex::showRectangles("show_last_rects", final_rects, srcBuff, show_final_rects, cvex::GREEN, 1);
	//cv::waitKey();
#endif // SHOW_LAST_RECTS

	if (final_rects.size() != 10)
	{
		cv::waitKey();
		//cv::Mat avr;
		//cvex::showRectangle("avrRect", avrRect, srcBuff, avr, cvex::RED, 0);
	}

	

	//框出ROI区域
	/*if (final_rects.empty())return;
	std::vector<cv::Point2f> saved_points;
	for (auto i = final_rects.begin(); i != final_rects.end(); ++i)
	{	
		cv::Point2f point[4];
		(*i).points(point);
		for (int k = 0; k < 4; ++k)
		{
			saved_points.push_back(point[k]);
		}
	}
	cv::RotatedRect RotaRect_buffRoi = cv::minAreaRect(saved_points);
	cv::Point2f points[4];
	RotaRect_buffRoi.points(points);
	std::vector<cv::Point2f>roi_4corners(points,points+4);
	//cv::Point2f* points1= roi_4corners.data;

	/*cv::Point2f roi_4corners[4];
	RotaRect_buffRoi.points(roi_4corners);

#ifdef SHOW_ROTATEDRECT_BUFFROI
	cv::Mat RotatedRect_buffRoi = srcBuff.clone();
	cvex::showRectangle("buffRoi", RotaRect_buffRoi, srcBuff, RotatedRect_buffRoi, cvex::GREEN, 1);
#endif // SHOW_ROTATEDRECT_BUFFROI

	//筛选出九宫格
	std::vector<cv::RotatedRect> buffnums;
	for (auto & roaRect : RotaRects_choose_nums)
	{
		//如果旋转矩形的中心点不在框出的roi的外包矩形区域里，则它肯定不是九宫格里的数字
		if (!RotaRect_buffRoi.boundingRect().contains(roaRect.center))
			continue;	

		if (cv::pointPolygonTest(roi_4corners, roaRect.center, false) != 1)
			continue;
		//cv::rectangle(srcBuff, RotaRect_buffRoi.boundingRect(), cvex::GREEN, 2);
		//cv::imshow("roi", srcBuff);
		buffnums.push_back(roaRect);
		//去除旁边的10个矩形和这10个矩形外的部分矩形，
		int low_than_base = 0;
		int high_than_base = 0;
		for (int i = 0; i < final_rects.size(); ++i)
		{	
			if (final_rects[i].center == roaRect.center)
			{
				buffnums.pop_back();
				break;
			}
				
		}	
	}
	/*for (auto i = buffnums.begin(); i != buffnums.end();)
	{

	}*/
	//cvex::showRectangles("buffnum", buffnums, srcBuff, srcBuff, cvex::GREEN, 0);
}

/*
int main()
{
	cv::VideoCapture capture("../Samples/buff1.mp4");
	if (!capture.isOpened()) return -1;
	while (capture.read(srcBuff)) 
	{
		count++;
		//if (count < 810)continue;
		if (srcBuff.empty()) continue;
		cv::Mat workImg = srcBuff.clone();
		cv::cvtColor(workImg, workImg, CV_BGR2GRAY);
		buff_FindRoi(workImg);
		cv::waitKey(1);
	}
	//srcBuff = cv::imread("buff.jpg");
	return 0;
}
*/