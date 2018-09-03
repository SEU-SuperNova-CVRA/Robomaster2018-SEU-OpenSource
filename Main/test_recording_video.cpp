#include"ImgProdCons.h"
#include"iostream"
#include<opencv2/opencv.hpp>
#include<thread>

using namespace std;
using namespace cv;

namespace rm
{
void ImgProdCons::init()
{
    //prevent usb port from being blocked
    init_signals();

    //Initialize camera
    _videoCapturePtr->open(0,2);
    _videoCapturePtr->setVideoFormat(640, 480, true);
    _videoCapturePtr->setExposureTime(100);
    _videoCapturePtr->setFPS(60);
    _videoCapturePtr->startStream();
    _videoCapturePtr->info();
}

void ImgProdCons::sense()
{

}

void ImgProdCons::consume()
{
    auto t1 = chrono::high_resolution_clock::now();
    VideoWriter writer;
    bool isRecording = false;
    Frame frame;
    for(;;)
    {
        if(!_buffer.getLatest(frame)) continue;

        imshow("capture", frame.img);
        if(isRecording)
        {
            writer << frame.img;
        }

        const int key = waitKey(1);
        if((char)key == 'c' || (char)key == 'C')
        {
            const string fileName = "/home/nvidia/Robomaster/Robomaster2018/2.avi";
            writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(frame.img.size().width, frame.img.size().height));
            if(!writer.isOpened())
            {
                cout << "Capture failed." << endl;
                continue;
            }
            isRecording = true;
            cout << "Start capture. " + fileName +" created." << endl;
        }
        else if((char)key == 'q')
        {
            break;
        }
        auto t2 = chrono::high_resolution_clock::now();
        cout << "process: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
        t1 = t2;
    }
}
}


