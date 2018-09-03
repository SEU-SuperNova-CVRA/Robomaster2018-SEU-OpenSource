#include"../General/numeric_rm.h"


namespace rmnum
{

std::vector<float> polyfit(const std::vector<float> &x, const std::vector<float> &y, unsigned int n)
{
	++n;
	std::vector<float> a;
	if(x.size() != y.size())
	{
		std::cout << "err: dimX != dimY" << std::endl;
		return 	a;
	}
	cv::Mat X(n, n, CV_32FC1);
	cv::Mat Y(1, n, CV_32FC1);
	cv::Mat A(n, 1, CV_32FC1);
	std::vector<float> tx;
	std::vector<float> ty;
	for(size_t i = 0; i <= n; ++i)
	{
		float tmpx = 0;
		float tmpy = 0;
		auto yi = y.begin();
		for(auto xi = x.begin(); xi != x.end(); ++xi, ++yi)
		{
			float tmp = pow(*xi, i);
			tmpx += tmp;
			tmpy += *yi*tmp;
		}
		tx.push_back(tmpx);
		ty.push_back(tmpy);
	}
	for(size_t i = n + 1; i <= 2 * n; ++i)
	{
		float tmpx = 0;
		for(auto xi = x.begin(); xi != x.end(); ++xi)
		{
			tmpx += pow(*xi, i);
		}
		tx.push_back(tmpx);
	}
	auto xi = tx.begin();
	for(size_t i = 0; i < n; ++i)
	{
		auto it = X.ptr<float>(i);
		for(size_t j = 0; j < n; j++)
		{
			*it++ = *xi++;
		}
		xi -= n - 1;
	}
	auto yi = ty.begin();
	for(auto it = Y.ptr<float>(0); yi != ty.end();)
	{
		*it++ = *yi++;
	}
	A = X.inv()*Y.t();
	for(size_t i = 0; i < n; ++i)
	{
		a.push_back(A.at<float>(n - i - 1, 0));
	}
	return a;
}

std::vector<float> polyval(const std::vector<float> &p, const std::vector<float> &x)
{
	std::vector<float> y;
	for(auto i = x.begin(); i != x.end(); ++i)
	{
		float tmpy = 0;
		for(auto j = p.begin(); j != p.end(); ++j)
		{
			tmpy += *j*pow(*i, p.end() - j - 1);
		}
		y.push_back(tmpy);
	}
	return y;
}
std::vector<float> polyval(const std::vector<float> &p, const std::vector<float> &x, float mean, float std)
{
	std::vector<float> y;
	for(auto i = x.begin(); i != x.end(); ++i)
	{
		float stdx = (*i - mean) / std;
		float tmpy = 0;
		for(auto j = p.begin(); j != p.end(); ++j)
		{
			tmpy += *j*pow(stdx, p.end() - j - 1);
		}
		y.push_back(tmpy);
	}
	return y;
}


}
