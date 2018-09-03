#include <unistd.h>

#include"iostream"
#include<opencv2/opencv.hpp>
#include<thread>
#include<time.h>

#include"ImgProdCons.h"
#include"./Pose/AngleSolver.hpp"
#include"./Rune/Rune.h"
#define DEBUG

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
    _videoCapturePtr->setExposureTime(70);
    _videoCapturePtr->setFPS(60);
    _videoCapturePtr->startStream();
    _videoCapturePtr->info();

    //Initilize serial
    _serialPtr->openPort();
    _serialPtr->setDebug(false);
    int self_color;
    while(_serialPtr->setup(self_color) != Serial::OJBK)
    {
        sleep(1);
    }
    cout << "I am " << (self_color == rm::BLUE ? "blue" : "red") << "." << endl;

    //Initialize angle solver
    AngleSolverParam angleParam;
    angleParam.readFile(8);//choose camera
    _solverPtr->init(angleParam);
    _solverPtr->setResolution(_videoCapturePtr->getResolution());

    //Initialize rune detector
    _runeDetectorPtr->init();

    _task = Serial::NO_TASK;
}

void ImgProdCons::sense()
{
    /* Loop for sensing */
    for(;;)
    {
        FeedBackData feedBackData;

        /* TODO: Handel exceptions when socket suddenly being plugged out. */
        if(_serialPtr->tryFeedBack(feedBackData, chrono::milliseconds(20)) == Serial::OJBK)
        {
            _task = feedBackData.task_mode;
            this_thread::sleep_for(chrono::milliseconds(80));
        }
        else
        {
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    }

}

void ImgProdCons::consume()
{
    /*
     * Variables for serials
     */
    ControlData controlData;


    /*
     *  Variables for angle solve module
     */
    int angleFlag;
    Vec2f targetAngle;

    /*
     * Variables for rune detector
     *
     */
    std::array<int, 9> runeNumbers;
    std::vector<int> ledNumbers;
    int runeFlag;
    int shootFlag;

    /*
     *  The main loop for armor detection
     */
    Frame frame;
    for(;;)
    {
#ifndef _MSC_VER
         if (_quit_flag)
            return;
#endif

        switch(_task)
        {
        case Serial::NO_TASK:
            cout<< "manual" <<endl;
            break;
        case Serial::SMALL_BUFF:
            cout<<"small buff"<<endl;
            break;
        case Serial::BIG_BUFF:
            cout<<"big buff"<<endl;
            break;
        case Serial::AUTO_SHOOT:
            cout<<"auto shoot"<<endl;
            break;
        default:
            cout<<"unknown mode"<<endl;
            break;
        }

        //Initialize rune detector
        if(_task == Serial::SMALL_BUFF)
        {
            _runeDetectorPtr->setMode(rm::MINI_RUNE, rm::RuneParam());
        }
        else if(_task == Serial::BIG_BUFF)
        {
            _runeDetectorPtr->setMode(rm::GREAT_RUNE, rm::RuneParam());
        }
        else
        {
            continue;
        }
        //auto t1 = chrono::high_resolution_clock::now();

        if(!_buffer.getLatest(frame)) continue;

        //First step: load img
        _runeDetectorPtr->loadImg(frame.img);

#ifdef DEBUG
        imshow("frameImg",frame.img);
        cv::waitKey(1);
#endif
        //Second step: detect and recognize rune
        runeFlag = _runeDetectorPtr->caculate();
        if(runeFlag == rm::RuneDetector::RUNE_NO)
        {
//            //this part for test
//            controlData.frame_seq   = frame.seq;
//            controlData.shoot_mode  = Serial::SINGLE_FIRE | Serial::HIGH_SPEED;
//            controlData.pitch_dev   = 5;
//            controlData.yaw_dev     = -5;
//            //try to send control data
//            if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) == Serial::OJBK)
//            {
//                //TODO:maybe need some time to let button fly
//                this_thread::sleep_for(chrono::milliseconds(1000));
//            }
//            else
//            {
//                cout<<"not sent"<<endl;
//            }
            continue;
        }

        //Third step: choose the target num's coordinate
        cv::Point2f target = _runeDetectorPtr->chooseTarget();

        shootFlag = _runeDetectorPtr->getFlag();
        //if success to get the target, shoot!
        if(shootFlag == rm::RuneDetector::SHOOT_YES)
        {
            // start angle solve
            if(_task == Serial::SMALL_BUFF)
            {
                _solverPtr->setTarget(target, rm::MINI_RUNE);
            }
            else if(_task == Serial::BIG_BUFF)
            {
                _solverPtr->setTarget(target, rm::GREAT_RUNE);
            }
            angleFlag = _solverPtr->solve();
            if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
            {
                targetAngle = _solverPtr->getAngle();
                //value the control data though caculating
                controlData.frame_seq   = frame.seq;
                controlData.shoot_mode  = Serial::SINGLE_FIRE | Serial::HIGH_SPEED;
                controlData.pitch_dev   = targetAngle[1];
                controlData.yaw_dev     = targetAngle[0];
                //try to send control data
                if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) == Serial::OJBK)
                {
                    //maybe need some time to let button fly
                    this_thread::sleep_for(chrono::milliseconds(800));
                    cout<<"success to sent"<<endl;
                }
                else
                {
                    cout<<"not sent"<<endl;
                }

               cout << "Deviation: " << targetAngle << endl;
            }

#ifdef DEBUG
            //show the target num in the frame
            cv::circle(frame.img, target, 5, cvex::CYAN, 3);
            cv::imshow("target",frame.img);
            cv::waitKey(1);
#endif

        }
        //if this time is not suit for shotting,continue
        else
        {
            continue;
        }
#ifdef DEBUG
        //show ledNumbers and runeNumbers
        runeNumbers = _runeDetectorPtr->getRuneNumbers();
        ledNumbers = _runeDetectorPtr->getLedNumbers();
        for(int i : ledNumbers)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;
        for(int i = 1; i <= 9; ++i)
        {
            std::cout << runeNumbers[i - 1] << " ";
            if(i % 3 == 0) std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << std::endl;
////        cv::waitKey(0);
#endif // DEBUG


       // auto t2 = chrono::high_resolution_clock::now();

        if(waitKey(1) == 'q')
        {
            return;
        }
     //   cout << "Detection duration: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
    }
}
}

