#include"ArmorDetector.h"
#include<iostream>
#include<vector>
#include<math.h>
#include<string>

#include"../General/opencv_extended.h"
#include"../General/numeric_rm.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace rm
{
/*
*	To make sure that -45 <= angle < 45
*/
enum
{
	WIDTH_GREATER_THAN_HEIGHT,
	ANGLE_TO_UP
};
cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
{
	using std::swap;

	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

	if(mode == WIDTH_GREATER_THAN_HEIGHT)
	{
		if(width < height)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	while(angle >= 90.0) angle -= 180.0;
	while(angle < -90.0) angle += 180.0;

	if(mode == ANGLE_TO_UP)
	{
		if(angle >= 45.0)
		{
			swap(width, height);
			angle -= 90.0;
		}
		else if(angle < -45.0)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	return rec;
}


ArmorDescriptor::ArmorDescriptor()
{
	rotationScore = 0;
	sizeScore = 0;
	vertex.resize(4);
	for(int i = 0; i < 4; i++)
	{
		vertex[i] = cv::Point2f(0, 0);
	}
	type = UNKNOWN_ARMOR;
}

ArmorDescriptor::ArmorDescriptor(const LightDescriptor & lLight, const LightDescriptor & rLight, const int armorType, const cv::Mat & grayImg, float rotaScore, ArmorParam _param)
{
	//handle two lights
	lightPairs[0] = lLight.rec();
	lightPairs[1] = rLight.rec();

	cv::Size exLSize(int(lightPairs[0].size.width), int(lightPairs[0].size.height * 2));
	cv::Size exRSize(int(lightPairs[1].size.width), int(lightPairs[1].size.height * 2));
	cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
	cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);

	cv::Point2f pts_l[4];
	exLLight.points(pts_l);
	cv::Point2f upper_l = pts_l[2];
	cv::Point2f lower_l = pts_l[3];

	cv::Point2f pts_r[4];
	exRLight.points(pts_r);
	cv::Point2f upper_r = pts_r[1];
	cv::Point2f lower_r = pts_r[0];

	vertex.resize(4);
	vertex[0] = upper_l;
	vertex[1] = upper_r;
	vertex[2] = lower_r;
	vertex[3] = lower_l;

	//set armor type
	type = armorType;

	//get front view
	getFrontImg(grayImg);
	rotationScore = rotaScore;

	// calculate the size score
	float normalized_area = contourArea(vertex) / _param.area_normalized_base;
	sizeScore = exp(normalized_area);

	// calculate the distance score
	Point2f srcImgCenter(grayImg.cols / 2, grayImg.rows / 2);
	float sightOffset = cvex::distance(srcImgCenter, cvex::crossPointOf(array<Point2f, 2>{vertex[0],vertex[2]}, array<Point2f, 2>{vertex[1],vertex[3]}));
	distScore = exp(-sightOffset / _param.sight_offset_normalized_base);

}

void ArmorDescriptor::getFrontImg(const Mat& grayImg)
{
	using cvex::distance;
	const Point2f&
		tl = vertex[0],
		tr = vertex[1],
		br = vertex[2],
		bl = vertex[3];

	int width, height;
	if(type == BIG_ARMOR)
	{
		width = 92;
		height = 50;
	}
	else
	{
		width = 50;
		height = 50;
	}

	Point2f src[4]{Vec2f(tl), Vec2f(tr), Vec2f(br), Vec2f(bl)};
	Point2f dst[4]{Point2f(0.0, 0.0), Point2f(width, 0.0), Point2f(width, height), Point2f(0.0, height)};
	const Mat perspMat = getPerspectiveTransform(src, dst);
	cv::warpPerspective(grayImg, frontImg, perspMat, Size(width, height));
}


ArmorDetector::ArmorDetector()
{
    _flag = ARMOR_NO;
    _roi = Rect(cv::Point(0, 0), _srcImg.size());
    _isTracking = false;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
    _debugWindowName = "debug info";
#endif // DEBUG_DETECTION
}

ArmorDetector::ArmorDetector(const ArmorParam & armorParam)
{
	_param = armorParam;
	_flag = ARMOR_NO;
	_roi = Rect(cv::Point(0, 0), _srcImg.size());
	_isTracking = false;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
	_debugWindowName = "debug info";
#endif // DEBUG_DETECTION
}

void ArmorDetector::init(const ArmorParam &armorParam)
{
    _param = armorParam;
}

void ArmorDetector::loadImg(const cv::Mat & srcImg)
{
	_srcImg = srcImg;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
	_debugImg = srcImg.clone();
#endif // DEBUG_DETECTION

	Rect imgBound = Rect(cv::Point(0, 0), _srcImg.size());

    if(_flag == ARMOR_LOCAL && _trackCnt != _param.max_track_num)
    {
        cv::Rect bRect = boundingRect(_targetArmor.vertex) + _roi.tl();
        bRect = cvex::scaleRect(bRect, Vec2f(3,2));	//以中心为锚点放大2倍
        _roi = bRect & imgBound;
        _roiImg = _srcImg(_roi).clone();
    }
    else
    {
		_roi = imgBound;
		_roiImg = _srcImg.clone();
		_trackCnt = 0;
    }

#ifdef DEBUG_DETECTION
	rectangle(_debugImg, _roi, cvex::YELLOW);
	//imshow(_debugWindowName, _debugImg);
#endif // DEBUG_DETECTION
}

//int thres = 190;
//static void on_th(int th, void *)
//{
//	thres = th;
//}

int ArmorDetector::detect()
{
	/*
	*	Detect lights and build light bars' desciptors
	*/
	_armors.clear();
	std::vector<LightDescriptor> lightInfos;
	{
		//cv::createTrackbar("threshold", "brightness_binary", &thres, 255, on_th);

		/*
		*	pre-treatment
		*/
		cv::Mat binBrightImg;
        cvtColor(_roiImg, _grayImg, COLOR_BGR2GRAY, 1);
		cv::threshold(_grayImg, binBrightImg, _param.brightness_threshold, 255, cv::THRESH_BINARY);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        dilate(binBrightImg, binBrightImg, element);

    //#ifdef DEBUG_PRETREATMENT
        //imshow("brightness_binary", binBrightImg);
        //waitKey();
    //#endif // DEBUG_PRETREATMENT

		/*
		*	find and filter light bars
		*/
		vector<vector<Point>> lightContours;
		cv::findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		for(const auto& contour : lightContours)
		{
			float lightContourArea = contourArea(contour);
			if(contour.size() <= 5 ||
			   lightContourArea < _param.light_min_area) continue;

			RotatedRect lightRec = fitEllipse(contour);
			adjustRec(lightRec, ANGLE_TO_UP);

			//float solidity = lightContourArea / lightRec.size.area();
            if(lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
			   lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity) continue;

			//Mat temp;
			//cvex::showRectangle("light_right_position", _srcImg, temp, lightRec, cvex::GREEN,0, _roi.tl());

			lightRec.size.width *= _param.light_color_detect_extend_ratio;
			lightRec.size.height *= _param.light_color_detect_extend_ratio;
			Rect lightRect = lightRec.boundingRect();
			const Rect srcBound(Point(0, 0), _roiImg.size());
			lightRect &= srcBound;
			Mat lightImg = _roiImg(lightRect);
			Mat lightMask = Mat::zeros(lightRect.size(), CV_8UC1);
			Point2f lightVertexArray[4];
			lightRec.points(lightVertexArray);
			std::vector<Point> lightVertex;
			for(int i = 0; i < 4; i++)
			{
				lightVertex.emplace_back(Point(lightVertexArray[i].x - lightRect.tl().x,
											   lightVertexArray[i].y - lightRect.tl().y));
			}
			fillConvexPoly(lightMask, lightVertex, 255);

            if(lightImg.size().area() <= 0 || lightMask.size().area() <= 0) continue;
            cv::dilate(lightMask, lightMask, element);
			const Scalar meanVal = mean(lightImg, lightMask);

			//Mat debugColorImg = _srcImg.clone();
			//const String BGR = "(" + std::to_string(int(meanVal[0])) + ", " + std::to_string(int(meanVal[1])) + ", " + std::to_string(int(meanVal[2])) + ")";

			if(((_enemy_color == BLUE) && (meanVal[BLUE] - meanVal[RED] > 20.0)) || (_enemy_color == RED && meanVal[RED] - meanVal[BLUE] > 20.0))
			{
				lightInfos.push_back(LightDescriptor(lightRec));

				//putText(debugColorImg, BGR, Point(lightVertexArray[0]) + _roi.tl(), FONT_HERSHEY_SIMPLEX, 0.4, cvex::GREEN, 1); //fontScalar 0.34
			}
			//else
			//{
			//	putText(debugColorImg, BGR, Point(lightVertexArray[0]) + _roi.tl(), FONT_HERSHEY_SIMPLEX, 0.4, cvex::CYAN, 1); //fontScalar 0.34
			//}
			//imshow("BGR", debugColorImg);
			//waitKey(0);

		}

	#ifdef DEBUG_DETECTION
		vector<RotatedRect> lightsRecs;
		for(auto & light : lightInfos)
		{
			lightsRecs.emplace_back(light.rec());
		}
        cvex::showRectangles(_debugWindowName, _debugImg, _debugImg, lightsRecs, cvex::MAGENTA, 1, _roi.tl());
	#endif //DEBUG_DETECTION

		if(lightInfos.empty())
		{
			return _flag = ARMOR_NO;
		}
	}


	/*
	*	find and filter light bar pairs
	*/

	{
		sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2)
		{
			return ld1.center.x < ld2.center.x;
		});
		vector<int> minRightIndices(lightInfos.size(), -1);
		for(size_t i = 0; i < lightInfos.size(); i++)
		{
			for(size_t j = i + 1; (j < lightInfos.size()); j++)
			{
				const LightDescriptor& leftLight  = lightInfos[i];
				const LightDescriptor& rightLight = lightInfos[j];

			#if (defined DEBUG_DETECTION) && (defined DEBUG_PAIR)
				Mat pairImg = _debugImg.clone();
				vector<RotatedRect> curLightPair{leftLight.rec(), rightLight.rec()};
                cvex::showRectangles("debug pairing",  pairImg, pairImg,curLightPair, cvex::CYAN, 1, _roi.tl());
			#endif // DEBUG_DETECTION && DEBUG_PAIR

				/*
				*	此处只针对2-3M以内情况较好的判断
				*	形态相似：
				*		平行
				*		height大小相似
				*/
				float angleDiff_ = abs(leftLight.angle - rightLight.angle);
				float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
				if(angleDiff_ > _param.light_max_angle_diff_ ||
				   LenDiff_ratio > _param.light_max_height_diff_ratio_)
				{
					continue;
				}

				/*
				*	相对位置合适:
				*		中心点y方向差距不大
				*		长宽比在范围内
				*/
				float dis = cvex::distance(leftLight.center, rightLight.center);
				float meanLen = (leftLight.length + rightLight.length) / 2;
				float yDiff = abs(leftLight.center.y - rightLight.center.y);
				float yDiff_ratio = yDiff / meanLen;
				float xDiff = abs(leftLight.center.x - rightLight.center.x);
				float xDiff_ratio = xDiff / meanLen;
				float ratio = dis / meanLen;
				if(yDiff_ratio > _param.light_max_y_diff_ratio_ ||
				   xDiff_ratio < _param.light_min_x_diff_ratio_ ||
				   ratio > _param.armor_max_aspect_ratio_ ||
				   ratio < _param.armor_min_aspect_ratio_)
				{
					continue;
				}

				// calculate pairs' info 
				int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
				// calculate the rotation score
				// rotation 正比于 ratio_off & y_off
				float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio - ratio, float(0)) : max(_param.armor_small_armor_ratio - ratio, float(0));
				float yOff = yDiff / meanLen;
				float rotationScore = ratiOff * ratiOff + yOff * yOff;

				ArmorDescriptor armor(leftLight, rightLight, armorType, _grayImg, rotationScore, _param);
				_armors.emplace_back(armor);
				break;
			}
		}

	#ifdef  DEBUG_DETECTION
		vector<vector<Point>> armorVertexs;
		for(const auto & armor : _armors)
		{
			vector<Point> intVertex;
			for(const auto& point : armor.vertex)
			{
				intVertex.emplace_back(Point(point.x,point.y));
			}
			armorVertexs.emplace_back(intVertex);
		}
        cvex::showContours(_debugWindowName, _debugImg, _debugImg,armorVertexs,  cvex::WHITE, 1, _roi.tl());
	#endif //  DEBUG_DETECTION
		
		if(_armors.empty())
		{
			return _flag = ARMOR_NO;
		}
	}

#ifdef GET_ARMOR_PIC
	_allCnt++;
	int i = 0;
	for(const auto & armor : _armors)
    {
        Mat regulatedFrontImg = armor.frontImg;
        if(armor.type == BIG_ARMOR)
        {
            regulatedFrontImg = regulatedFrontImg(Rect(21, 0, 50, 50));
        }
        imwrite("/home/nvidia/Documents/ArmorTrainingSample/" + to_string(_allCnt) + "_" + to_string(i) + ".bmp", regulatedFrontImg);
		i++;
	}
#endif // GET_ARMOR_PIC

	//delete the fake armors
	_armors.erase(remove_if(_armors.begin(), _armors.end(), [](ArmorDescriptor& i)
	{
		return !(i.isArmorPattern());
	}), _armors.end());

	if(_armors.empty())
	{
		_targetArmor.clear();

		if(_flag == ARMOR_LOCAL)
		{
			//cout << "Tracking lost" << endl;
			return _flag = ARMOR_LOST;
		}
		else
		{
			//cout << "No armor pattern detected." << endl;
			return _flag = ARMOR_NO;
		}
	}

	//calculate the final score
	for(auto & armor : _armors)
	{
		armor.finalScore = armor.sizeScore + armor.distScore + armor.rotationScore;
	}

	//choose the one with highest score, store it on _targetArmor
	std::sort(_armors.begin(), _armors.end(), [](const ArmorDescriptor & a, const ArmorDescriptor & b)
	{
		return a.finalScore > b.finalScore;
	});
	_targetArmor = _armors[0];

	//update the flag status	
	_trackCnt++;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
	vector<Point> intVertex;
	for(const auto& point : _targetArmor.vertex)
	{
		Point fuckPoint = point;
		intVertex.emplace_back(fuckPoint);
	}
    cvex::showContour(_debugWindowName, _debugImg, _debugImg, intVertex, cvex::GREEN, -1, _roi.tl());
#endif //DEBUG_DETECTION

	//std::cout << "armor detected" << endl;
	return _flag = ARMOR_LOCAL;
}

bool ArmorDescriptor::isArmorPattern() const
{
//    Mat regulatedImg;
//    //TODO: 只截取中间部分，对大装甲板要同样有效
//    if(type == BIG_ARMOR)
//    {
//        regulatedImg = frontImg(Rect(21, 0, 50, 50));
//    }
//    else
//    {
//        regulatedImg = frontImg;
//    }

//    resize(regulatedImg,regulatedImg, Size(regulatedImg.size().width / 2, regulatedImg.size().height / 2));
//    // copy the data to make the matrix continuous
//    Mat temp;
//    regulatedImg.copyTo(temp);
//    Mat data = temp.reshape(1, 1);

//    data.convertTo(data, CV_32FC1);

//    Ptr<SVM> svm = StatModel::load<SVM>("/home/nvidia/Robomaster/Robomaster2018/Armor/SVM3.xml");


//    int result = (int)svm->predict(data);
//    if(result == 1) return true;
//    else return false;
    return true;
}


const std::vector<cv::Point2f> ArmorDetector::getArmorVertex() const
{
	vector<cv::Point2f> realVertex;
	for(int i = 0; i < 4; i++)
	{
		realVertex.emplace_back(Point2f(_targetArmor.vertex[i].x + _roi.tl().x, 
										_targetArmor.vertex[i].y + _roi.tl().y));
	}

//#ifdef DEBUG_DETECTION
//	for(int i = 0; i < 3; i++)
//	{
//		cv::line(_srcImg, realVertex[i], realVertex[i + 1], cvex::BLUE);
//	}
//	cv::line(_srcImg, realVertex[3], realVertex[0], cvex::BLUE);
//	imshow("vertex", _srcImg);
//#endif // DEBUG_DETECTION

	return realVertex;
}

int ArmorDetector::getArmorType() const
{
	return _targetArmor.type;
}

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
void ArmorDetector::showDebugImg() const
{
	imshow(_debugWindowName, _debugImg);
}
Mat ArmorDetector::getDebugImg()
{
	return _debugImg;
}
#endif // DEBUG_DETECTION || SHOW_RESULT

}
