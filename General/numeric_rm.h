/* *********************************************************
Description:
	frame_counteric operations and comparisons
	数值操作和比较
Authors:
	Binyan Hu, <binyan_hu@qq.com>
	Renjie Cui, <773987208@qq.com>
Create Date:
	2017 - 10 - 31
History:
	2018 - 01 - 26 添加polyval polyfit函数

************************************************************ */
#pragma once

#include<opencv2/opencv.hpp>
#include<numeric>
#include <vector>
#include<array>

namespace rmnum
{
/*
	val1 approximately equal to val2 within relative tolerance
	相对误差约等于
*/
template<typename InputValType1, typename InputValType2, typename ToleranceType>
inline bool relApproxEqual(const InputValType1 val1, const InputValType2 val2, const ToleranceType relTolerance)
{
	return(abs(val1 - val2) < val2*relTolerance || abs(val1 - val2) < val1*relTolerance);
}

/*
	val1 approximately equal to val2 within absolute tolerance
	绝对误差约等于
*/
template<typename InputValType1, typename InputValType2, typename ToleranceType>
inline bool absApproxEqual(const InputValType1 val1, const InputValType2 val2, const ToleranceType absTolerance)
{
	return(abs(val1 - val2) < absTolerance);
}

template<typename InputValType1, typename InputValType2, typename ToleranceType>
inline bool absApproxEqual(const InputValType1 val1, const InputValType2 val2, const ToleranceType absToleranceMin, const ToleranceType absToleranceMax)
{
	return(abs(val1 - val2) < absToleranceMax && abs(val1 - val2) > absToleranceMin);
}
/*
	get mean of a frame_countber array
	数组的均值
*/
template<typename ValType>
inline ValType mean(const std::vector<ValType>& arr)
{
	return(accumulate(arr.begin(), arr.end(), 0) / arr.size());
}

/*
	多项式曲线拟合，与matlab的同名函数功能相同
*/
std::vector<float> polyfit(const std::vector<float> &x, const std::vector<float> &y, unsigned int n);

/*
	多项式数值计算，与matlab的同名函数功能相同，p为多项式系数（降幂），mean为x的均值（中心化时使用），std为标准差（中心化时使用）
*/
std::vector<float> polyval(const std::vector<float> &p, const std::vector<float> &x);
std::vector<float> polyval(const std::vector<float> &p, const std::vector<float> &x, float mean, float std);

}

