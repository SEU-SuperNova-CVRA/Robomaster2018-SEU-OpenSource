#include <unistd.h>

#include"iostream"
#include<opencv2/opencv.hpp>
#include<thread>
#include<time.h>

#include"ImgProdCons.h"
#include"./Pose/AngleSolver.hpp"
#include"./Armor/ArmorDetector.h"
#include"./Rune/Rune.h"
//#define DEBUG
#define CAMERA_NUMBER 8
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
    _videoCapturePtr->setVideoFormat(1280, 720, true);
    _videoCapturePtr->setExposureTime(100);
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
    angleParam.readFile(CAMERA_NUMBER);//choose camera
    _solverPtr->init(angleParam);
    _solverPtr->setResolution(_videoCapturePtr->getResolution());

    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetectorPtr->init(armorParam);
    _armorDetectorPtr->setEnemyColor(self_color == rm::BLUE ? rm::RED : rm::BLUE);

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
     * Variables for recording camera
     */
    VideoWriter writer;
    bool isRecording = false;
    bool readyTOrecord = false;

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

    /* Variables for armor detector module */
    int armorFlag;
    int armorType;
    std::vector<cv::Point2f> armorVertex;
    /*
     *  The main loop for task
     */
    Frame frame;
    for(;;)
    {
#ifndef _MSC_VER
        if (_quit_flag)
            return;
#endif
        if(_serialPtr->getErrorCode() == Serial::SYSTEM_ERROR || !_videoCapturePtr->isOpened())
        {
            this_thread::sleep_for(chrono::seconds(3));
        }
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

        if(!_buffer.getLatest(frame)) continue;



        if(_task == Serial::SMALL_BUFF)
        {
            /* Reinitializations */
            _videoCapturePtr->setExposureTime(70);
            frame.img = frame.img(cv::Rect(60, 0, 960, 720));
            resize(frame.img,frame.img,Size(640, 480));

            isRecording = false;

            _solverPtr->setResolution(Size(640, 480));

            _runeDetectorPtr->setMode(rm::MINI_RUNE, rm::RuneParam());

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
                    if(CAMERA_NUMBER == 6)
                    {
                        controlData.pitch_dev   = targetAngle[1] ;
                        controlData.yaw_dev     = targetAngle[0] ;
                    }
                    if(CAMERA_NUMBER == 8)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 3.26;
                        controlData.yaw_dev     = targetAngle[0] - 9.48;
                    }

                    //try to send control data
                    if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) == Serial::OJBK)
                    {
                        //maybe need some time to let button fly
                        this_thread::sleep_for(chrono::milliseconds(1000));
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
        }
        else if(_task == Serial::BIG_BUFF)
        {
            _videoCapturePtr->setExposureTime(70);
            isRecording = false;
            _runeDetectorPtr->setMode(rm::GREAT_RUNE, rm::RuneParam());

            _solverPtr->setResolution(Size(640, 480));


        }
        else if(_task ==Serial::AUTO_SHOOT)
        {
            //            _videoCapturePtr->setVideoFormat(1280, 720, true);
            _videoCapturePtr->setExposureTime(100);
            isRecording = false;
            readyTOrecord = true;

            _solverPtr->setResolution(Size(1280, 720));

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
                    targetAngle = _solverPtr->getAngle();

                    controlData.frame_seq   = frame.seq;
                    controlData.shoot_mode  = Serial::BURST_FIRE | Serial::LOW_SPEED;
                    controlData.pitch_dev   = targetAngle[1];
                    controlData.yaw_dev     = targetAngle[0];

                    if(CAMERA_NUMBER == 6)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 7.2;
                        controlData.yaw_dev     = targetAngle[0] + 2;
                    }
                    if(CAMERA_NUMBER == 7)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 1.7;
                        controlData.yaw_dev     = targetAngle[0] - 0.9;
                    }

                    if(CAMERA_NUMBER == 8)
                    {
                        controlData.pitch_dev   = targetAngle[1] - 4.81;
                        controlData.yaw_dev     = targetAngle[0] -1.75;
                    }

                    //                controlData.speed_on_rail = 0;

#ifdef SENTRY
                    controlData.gimbal_mode = Serial::SERVO_MODE;
#endif
                    if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                    {
                        cout<<"not sent"<<endl;
                    }

                    cout << "Deviation: " << "["<<controlData.pitch_dev << " " << controlData.yaw_dev<<"]" << endl;
                }
            }

#ifdef SENTRY
            else
            {
                controlData.gimbal_mode = Serial::PATROL_AROUND;
                if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                {
                    cout<<"not sent"<<endl;
                }

            }
#endif

            //cout << "Deviation: " << targetAngle << endl;

        }
        else
        {
            //            if(readyTOrecord)
            //            {
            //                _videoCapturePtr->setVideoFormat(640, 480, true);
            //                _videoCapturePtr->setExposureTime(200);

            //                if(isRecording)
            //                {
            //                    writer << frame.img;
            //                }
            //                else
            //                {
            //                    time_t t;
            //                    time(&t);
            //                    const string fileName = "/home/nvidia/Robomaster/Robomaster2018/" + to_string(t) + ".avi";
            //                    writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(frame.img.size().width, frame.img.size().height));
            //                    if(!writer.isOpened())
            //                    {
            //                        cout << "Capture failed." << endl;
            //                        continue;
            //                    }
            //                    isRecording = true;
            //                    cout << "Start capture. " + fileName +" created." << endl;
            //                }

            //            }
        }
        //auto t1 = chrono::high_resolution_clock::now();


        // auto t2 = chrono::high_resolution_clock::now();

#ifdef DEBUG
        if(waitKey(1) == 'q')
        {
            return;
        }
        //   cout << "Detection duration: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
#endif

    }
}
}
