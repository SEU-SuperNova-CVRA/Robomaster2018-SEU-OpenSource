/*
    Notice! when you use this file you have to
    commet the :

        if(_serialPtr->tryRecord(seq, chrono::milliseconds(3)) != Serial::OJBK)
        {
            continue;
        }
    on the ImgProdCons.cpp

    &&

        std::unique_ptr<Serial> _serialPtr;
    on the ImgProdCons.h
    and comment all the _serialPtr instance to handle the error

*/

#include<unistd.h>
#include"iostream"
#include<opencv2/opencv.hpp>
#include<thread>

#include"ImgProdCons.h"
#include"./Armor/ArmorDetector.h"
#include"./Pose/AngleSolver.hpp"

using namespace std;
using namespace cv;

namespace rm
{

void ImgProdCons::init()
{
    //prevent usb port from being blocked
    init_signals();


    //Initialize camera
    _videoCapturePtr->open(0,2); // 0 works fine on SHIT on big panel
    _videoCapturePtr->setVideoFormat(1280, 720, true);
    _videoCapturePtr->setExposureTime(100);
    _videoCapturePtr->setFPS(60);
    _videoCapturePtr->startStream();
    _videoCapturePtr->info();


    int self_color;
    self_color = rm::BLUE;

    //Initialize angle solver
    AngleSolverParam angleParam;
    angleParam.readFile(6);
    _solverPtr->init(angleParam);
    _solverPtr->setResolution(_videoCapturePtr->getResolution());


    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetectorPtr->init(armorParam);
    _armorDetectorPtr->setEnemyColor(self_color == rm::BLUE ? rm::RED : rm::BLUE);
}

void ImgProdCons::sense()
{

/*
    comment all this part if you want to run this test project correctlly

*/
    /* Loop for sensing */
    //for(;;)
//    {
//        FeedBackData feedBackData;

        /* TODO: Handel exceptions when socket suddenly being plugged out. */
//        if(_serialPtr->tryFeedBack(feedBackData, chrono::milliseconds(20)) == Serial::OJBK)
//        {
//            _task = feedBackData.task_mode;
//            this_thread::sleep_for(chrono::milliseconds(80));
//        }
//        else
//        {
//            this_thread::sleep_for(chrono::milliseconds(50));
//        }
//    }
}

void ImgProdCons::consume()
{
    /* Variables for serial communication*/

    /* Variables for angle solve module */
    int angleFlag;
    Vec2f targetAngle;
    /* Variables for armor detector modeule */
    int armorFlag;
    int armorType;
    std::vector<cv::Point2f> armorVertex;

    /* The main loop for armor detection */
    auto t1 = chrono::high_resolution_clock::now();
    Frame frame;
    for(;;)
    {

        if(!_buffer.getLatest(frame)) continue;

        _armorDetectorPtr->loadImg(frame.img);

        armorFlag = _armorDetectorPtr->detect();
        if(armorFlag == ArmorDetector::ARMOR_LOCAL || armorFlag == ArmorDetector::ARMOR_GLOBAL)
        {
            armorVertex = _armorDetectorPtr->getArmorVertex();
            armorType = _armorDetectorPtr->getArmorType();

            _solverPtr->setTarget(armorVertex, armorType);
            angleFlag = _solverPtr->solve();
            if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
            {
                targetAngle = _solverPtr->getCompensateAngle();

                cout << "Deviation: " << targetAngle << endl;
            }
        }

        auto t2 = chrono::high_resolution_clock::now();
        cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
        cout << endl;
        t1 = t2;

        _armorDetectorPtr->showDebugImg();
        if(waitKey(1) == 'q')
        {
            return;
        }

    }
}
}
