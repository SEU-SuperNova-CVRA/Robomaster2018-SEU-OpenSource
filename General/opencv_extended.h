#pragma once

#include<opencv2/opencv.hpp>
#include<vector>
#include<array>

namespace cv
{
/*
*	@Brief: Translate a coordination
*/
template<typename T1, typename T2>
inline Point_<T1> operator/(const cv::Point_<T1>& pt, const T2 den)
{
	return Point_<T1>(pt.x / den, pt.y / den);
}

typedef cv::Rect_<float> Rect2f;

/*
*	@Brief: Translate a rotated rectangle
*/
template<typename T>
cv::RotatedRect operator+(const cv::RotatedRect& rec, const cv::Point_<T>& pt)
{
	return cv::RotatedRect(cv::Point2f(rec.center.x + pt.x, rec.center.y + pt.y), rec.size, rec.angle);
}
}


namespace cvex
{
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar CYAN(255, 255, 0);
const cv::Scalar WHITE(255, 255, 255);

/*
*	@Brief: Get the euclidean distacne of two points
*/
template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2)
{
	return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

/*
*	@Brief: Get manhattan distance of two points, i.e., |△x+△y|.
*/
template<typename T>
float distanceManhattan(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2)
{
	return std::abs(pt1.x - pt2.x) + std::abs(pt1.y - pt2.y);
}

/*
*	@Brief: get the cross point of two lines
*/
template<typename ValType>
const cv::Point2f crossPointOf(const std::array<cv::Point_<ValType>, 2>& line1, const std::array<cv::Point_<ValType>, 2>& line2)
{
	ValType a1 = line1[0].y - line1[1].y;
	ValType b1 = line1[1].x - line1[0].x;
	ValType c1 = line1[0].x*line1[1].y - line1[1].x*line1[0].y;

	ValType a2 = line2[0].y - line2[1].y;
	ValType b2 = line2[1].x - line2[0].x;
	ValType c2 = line2[0].x*line2[1].y - line2[1].x*line2[0].y;

	ValType d = a1 * b2 - a2 * b1;

	if(d == 0.0)
	{
		return cv::Point2f(FLT_MAX, FLT_MAX);
	}
	else
	{
		return cv::Point2f(float(b1*c2 - b2 * c1) / d, float(c1*a2 - c2 * a1) / d);
	}
}
inline const cv::Point2f crossPointOf(const cv::Vec4f& line1, const cv::Vec4f& line2)
{
	const std::array<cv::Point2f, 2> line1_{cv::Point2f(line1[2],line1[3]),cv::Point2f(line1[2] + line1[0],line1[3] + line1[1])};
	const std::array<cv::Point2f, 2> line2_{cv::Point2f(line2[2],line2[3]),cv::Point2f(line2[2] + line2[0],line2[3] + line2[1])};
	return crossPointOf(line1_, line2_);
}

/*template<typename ValType>
const cv::Point2f crossPointOf(std::initializer_list<ValType> line1, std::initializer_list<ValType> line2)
{
	ValType a1 = line1[0].y - line1[1].y;
	ValType b1 = line1[1].x - line1[0].x;
	ValType c1 = line1[0].x*line1[1].y - line1[1].x*line1[0].y;

	ValType a2 = line2[0].y - line2[1].y;
	ValType b2 = line2[1].x - line2[0].x;
	ValType c2 = line2[0].x*line2[1].y - line2[1].x*line2[0].y;

	ValType d = a1 * b2 - a2 * b1;

	if (d == 0.0)
	{
		return cv::Point2f(0.0, 0.0);
	}

	return cv::Point2f(float(b1*c2 - b2 * c1) / d, float(c1*a2 - c2 * a1) / d);
}*/

/*
*	
*/
template<typename T1, typename T2>
const cv::Rect_<T1> scaleRect(const cv::Rect_<T1>& rect, const cv::Vec<T2,2> scale, cv::Point_<T1> anchor = cv::Point_<T1>(-1, -1))
{
	if(anchor == cv::Point_<T1>(-1, -1))
	{
		anchor = cv::Point_<T1>(rect.x + rect.width / 2, rect.y + rect.height / 2);
	}
	cv::Point_<T1> tl((cv::Vec<T1, 2>(rect.tl()) - cv::Vec<T1, 2>(anchor)).mul(scale) + cv::Vec<T1, 2>(anchor));
	cv::Size_<T1> size(rect.width*scale[0], rect.height*scale[1]);
	return cv::Rect_<T1>(tl, size);
}

/*
*	@Brief: shows the histogram of an image
*/
void showHist(const cv::Mat img);

void rotatedRectangle(cv::Mat& img, const cv::RotatedRect& rec, const cv::Scalar & color);

/*
*	@Brief: Draw a single contour
*	@Input: @contour: contour must be Mat or vector<Point>
*/
template<typename ContourType>
void showContour(const std::string& windowName,
				 const cv::Mat& srcImg,
				 cv::Mat&dstImg,
				 ContourType contour,
				 const cv::Scalar& color,
				 const int waitTime = -1,
				 const cv::Point offset = cv::Point(0, 0))
{
	if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
	else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

	cv::drawContours(dstImg, std::vector<ContourType>{contour}, -1, color, 1, 8, cv::noArray(), INT_MAX, offset);

	cv::imshow(windowName, dstImg);
	if(waitTime >= 0)
	{
		cv::waitKey(waitTime);
	}
}

/*
*	@Brief: Draw a contours
*	@Input: @contour: contour  must be Mat or vector<Point>
*/
template<typename ContoursType>
void showContours(const std::string& windowName, 
				  const cv::Mat& srcImg, 
				  cv::Mat&dstImg, 
				  const ContoursType& contours,  
				  const cv::Scalar& color, 
				  const int waitTime = -1, 
				  const cv::Point offset = cv::Point(0, 0))
{
	if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
	else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

	cv::drawContours(dstImg, contours, -1, color, 1, 8, cv::noArray(), INT_MAX, offset);

	cv::imshow(windowName, dstImg);
	if(waitTime >= 0)
	{
		cv::waitKey(waitTime);
	}
}

template <typename RecType, bool = std::is_pointer<RecType>::value>
struct showRectangle_wrapper
{};

template <typename RecType>
struct showRectangle_wrapper<RecType, true>
{
	static void showRectangle(const std::string& windowName, 
							  const cv::Mat& srcImg,
							  const cv::Mat& dstImg, 
							  RecType& rec,
							  const cv::Scalar& color,
							  const int waitTime = -1, 
							  const cv::Point offset = cv::Point(0, 0))
	{
		if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

		rotatedRectangle(dstImg, (*rec) + offset, color);

		cv::imshow(windowName, dstImg);
		if(waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}

	}
};

template <typename RecType>
struct showRectangle_wrapper<RecType, false>
{
	static void showRectangle(const std::string& windowName, 
							  const cv::Mat& srcImg,
							  cv::Mat& dstImg,
							  RecType& rec,
							  const cv::Scalar& color,
							  const int waitTime = -1, 
							  const cv::Point offset = cv::Point(0, 0))
	{
		if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

		rotatedRectangle(dstImg, rec + offset, color);

		cv::imshow(windowName, dstImg);
		if(waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}
	}

};

template <typename RecType>
void showRectangle(const std::string& windowName, 
				   const cv::Mat&srcImg,
				   cv::Mat& dstImg,
				   RecType& rec,
				   const cv::Scalar& color,
				   const int waitTime = -1,
				   const cv::Point offset = cv::Point(0, 0))
{
	using RecTypeWithoutRef = typename std::remove_reference<RecType>::type;
	showRectangle_wrapper<RecTypeWithoutRef>::showRectangle(windowName, srcImg, dstImg, rec, color, waitTime, offset);
}

template <typename RecsContainer, bool = std::is_pointer<RecsContainer>::value>
struct showRecs_wrapper
{};

template <typename Recs>
struct showRecs_wrapper<Recs, true>
{
	static void showRectangles(const std::string& windowName,
							   const cv::Mat& srcImg, 
							   cv::Mat& dstImg,
							   Recs& recs_or_recPtrs,
							   const cv::Scalar& color,
							   const int waitTime = -1,
							   const cv::Point offset = cv::Point(0, 0))
	{
		if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

		for(const auto& rec : recs_or_recPtrs)
		{

			rotatedRectangle(dstImg, (*rec) + offset, color);
		}
		cv::imshow(windowName, dstImg);
		if(waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}

	}
};

template <typename Recs>
struct showRecs_wrapper<Recs, false>
{
	static void showRectangles(const std::string& windowName,
							   const cv::Mat& srcImg, 
							   cv::Mat& dstImg,
							   Recs& recs_or_recPtrs,
							   const cv::Scalar& color,
							   const int waitTime = -1,
							   const cv::Point offset = cv::Point(0, 0))
	{
		if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

		for(const auto& rec : recs_or_recPtrs)
		{
			rotatedRectangle(dstImg, rec + offset, color);
		}
		cv::imshow(windowName, dstImg);
		if(waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}

	}
};


template <typename RecsContainer>
void showRectangles(const std::string& windowName,
					const cv::Mat& srcImg, 
					cv::Mat& dstImg,
					RecsContainer& recs_or_recPtrs,
					const cv::Scalar& color,
					const int waitTime = -1,
					const cv::Point offset = cv::Point(0, 0))
{
	using RecsWithoutRef = typename std::remove_reference<RecsContainer>::type;
	showRecs_wrapper<RecsWithoutRef>::showRectangles(windowName, srcImg, dstImg, recs_or_recPtrs, color, waitTime, offset);

}

/*
template<typename RecsContainer>
void showRectangles(const std::string& windowName,
					RecsContainer&& recs_or_recPtrs,
					const cv::Mat& srcImg, cv::Mat& dstImg,
					const cv::Scalar& color = cv::Scalar(255, 255, 255),
					const int waitTime = -1,
					const cv::Point offset = cv::Point(0, 0))
{
	if (srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
	else if (srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

	for (const auto& rec : recs_or_recPtrs)
	{
		if constexpr (std::is_pointer_v<decltype(rec)>)
		{
			rotatedRectangle(dstImg, (*rec) + offset, color);
		}
		else
		{
			rotatedRectangle(dstImg, rec + offset, color);
		}
	}
	cv::imshow(windowName, dstImg);
	if (waitTime >= 0)
	{
		cv::waitKey(waitTime);
	}
}
*/

template<typename T>
void drawCrossing(const cv::Mat& srcImg, 
				  cv::Mat& dstImg, 
				  const cv::Point_<T>& pt, 
				  const cv::Scalar& color)
{
	if(srcImg.channels() == 1)	cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
	else if(srcImg.data != dstImg.data) srcImg.copyTo(dstImg);

	typedef cv::Vec<T, 2> Vec_t;
	typedef cv::Point_<T> Point_t;
	cv::line(dstImg, Point_t(Vec_t(pt) + Vec_t(-10, 0)), Point_t(Vec_t(pt) + Vec_t(10, 0)), color,2);
	cv::line(dstImg, Point_t(Vec_t(pt) + Vec_t(0, -10)), Point_t(Vec_t(pt) + Vec_t(0, 10)), color,2);
}


}
