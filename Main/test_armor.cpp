#include"ImgProdCons.h"
#include"iostream"
#include<opencv2/opencv.hpp>

#include"./Armor/ArmorDetector.h"

using namespace std;
using namespace cv;

namespace rm
{

ArmorDetector _armorDetector;
void ImgProdCons::init()
{
    //prevent usb port from being blocked
    init_signals();

    //Initialize camera
    _videoCapture.open(1,2);
    _videoCapture.setVideoFormat(1280, 720, true);
    _videoCapture.setExposureTime(100);
    _videoCapture.setFPS(60);
    _videoCapture.startStream();
    _videoCapture.info();

    //	_videoCapture.open(0);
    //	_videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    //	_videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);


    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetector.init(armorParam);
    _armorDetector.setEnemyColor(rm::BLUE);
}

void ImgProdCons::consume()
{
    /*
     *  Variables for armor detector modeule
     */
    int armorFlag;
    int armorType;
    std::vector<cv::Point2f> armorVertex;

    /*
     *  The main loop
     */
    Frame frame;
    for(;;)
    {
        auto t1 = chrono::high_resolution_clock::now();

        if(!getLatest(frame)) continue;

        _armorDetector.loadImg(frame.img);
        armorFlag = _armorDetector.detect();
        if(armorFlag == ArmorDetector::ARMOR_LOCAL || armorFlag == ArmorDetector::ARMOR_GLOBAL)
        {
            armorVertex = _armorDetector.getArmorVertex();
            armorType = _armorDetector.getArmorType();

            if(armorType == BIG_ARMOR)
            {
                cout<<"big"<<endl;
            }
            else if(armorType == SMALL_ARMOR)
            {
                cout<<"small"<<endl;
            }

        }

        _armorDetector.showDebugImg();

        if(waitKey(1) == 'q')
        {
            return;
        }

        auto t2 = chrono::high_resolution_clock::now();
        cout << "process: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
        t1 = t2;
    }
}
}
