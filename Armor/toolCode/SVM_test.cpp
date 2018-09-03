#include<opencv2\opencv.hpp>
#include<opencv2\ml.hpp>
#include<iostream>
#include<chrono>
using namespace  std;
using namespace cv;
using namespace cv::ml;


int main() 
{

	string filePath = "D:/possible_armor_area/N_4/";
	for (int j = 1; j <= 938; j++)
	{
		for (int i = 0; i < 5; i++)
		{
			// safely read the img
			string fileName = to_string(j) + "_" + to_string(i) + ".jpg";
			Mat src = imread(filePath + fileName, IMREAD_GRAYSCALE);
			if (src.empty()) continue;
			if (src.size().width > 50)
				continue;

			imshow("src", src);
			waitKey(0);

			auto t1 = chrono::high_resolution_clock::now();

			Mat data = src.reshape(1, 1);
			data.convertTo(data, CV_32FC1);

			Ptr<SVM> svm = StatModel::load<SVM>("C:/Users/cooper/Desktop/Resp_for_ML/ML-method-for-Judging-Armor-Pattern/SVM.xml");
			int result = (int)svm->predict(data);
			if (result == 1)
			{
				cout << "it is a Armor" << endl;
			}
			else
			{
				cout << "Not a Armor!" << endl;
			}

			auto t2 = chrono::high_resolution_clock::now();
			cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
			cout << endl;
		}
	}

	system("pause");
}

