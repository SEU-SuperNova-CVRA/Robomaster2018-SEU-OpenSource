#include<iostream>
#include<opencv2\opencv.hpp>
#include<Windows.h>

using namespace std;
using namespace cv;

void onSharpChange(int sharp, void* capture)
{
	((cv::VideoCapture*)capture)->set(cv::CAP_PROP_SHARPNESS, sharp);
}
void onExpChange(int exposure, void* capture)
{
	((cv::VideoCapture*)capture)->set(cv::CAP_PROP_EXPOSURE, exposure - 10);
	cout << "set exposre to: " << exposure - 10 << endl;
}

int main()
{
	VideoCapture cap(CAP_DSHOW +0);
	if (!cap.isOpened()) return -1;
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	int fps = cap.get(CAP_PROP_FPS);
	int width = cap.get(CAP_PROP_FRAME_WIDTH);
	int height = cap.get(CAP_PROP_FRAME_HEIGHT);

	bool isRecording = false;
	//VideoWriter writer("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(640, 480));
	VideoWriter writer;

	int exposure = 0, sharpness = 0;
	cv::namedWindow("capture");
	cv::createTrackbar("exposure", "capture", &exposure, 5, onExpChange, &cap);
	cv::createTrackbar("sharpness", "capture", &sharpness, 5, onSharpChange, &cap);
	cout << "**************************" << endl;
	cout << "press c to start the record" << endl;
	cout << "press q to quit the record" << endl;
	cout << "**************************" << endl;
	Mat frame;
	while (cap.read(frame))
	{
		imshow("capture", frame);

		if (isRecording)
		{
			writer << frame;
		}

		const int key = waitKey(1);
		if ((char)key == 'c')
		{
			writer.open(to_string(GetTickCount()) + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(width, height));
			if (!writer.isOpened())
			{
				cout << "capture failed." << endl;
				continue;
			}
			isRecording = true;
			cout << "start capturing." << endl;
		}
		else if ((char)key == 'q')
		{
			break;
		}
	}

	//Mat white = imread("D:/possible_armor_area/set1 blue/19_0.bmp", 0);
	//Mat black = imread("D:/possible_armor_area/set1 blue/22_0.bmp", 0);
	//threshold(white, white, 200, 255, CV_THRESH_OTSU);
	//threshold(black, black, 200, 255, CV_THRESH_OTSU);
	//medianBlur(white, white, 3);
	//medianBlur(black, black, 3);
	//imshow("white", white);
	//imshow("black", black);
	//waitKey(0);
	return 0;
}