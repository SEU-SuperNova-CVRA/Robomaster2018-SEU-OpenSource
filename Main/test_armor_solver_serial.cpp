#include<unistd.h>
#include"iostream"
#include<opencv2/opencv.hpp>
#include<thread>
#include<list>

#include"ImgProdCons.h"
#include"./Armor/ArmorDetector.h"
#include"./Pose/AngleSolver.hpp"

using namespace std;
using namespace cv;

#define SENTRY
#define DEBUG

namespace rm
{

void ImgProdCons::init()
{
    //prevent usb port from being blocked
    init_signals();


    //Initialize camera
    _videoCapturePtr->open(0,2); // 0 works fine for small panel , if you meet problem opening the camera, try change this

    _videoCapturePtr->setVideoFormat(1280, 720, true);
    _videoCapturePtr->setExposureTime(200);
    _videoCapturePtr->setFPS(60);
    _videoCapturePtr->startStream();
    _videoCapturePtr->info();
\

    //Initilize serial
    _serialPtr->openPort();
    _serialPtr->setDebug(true);
    int self_color;
//    self_color = rm::RED;
    while(_serialPtr->setup(self_color) != Serial::OJBK)
    {
        sleep(1);
    }
    cout << "I am " << (self_color == rm::BLUE ? "blue" : "red") << "." << endl;


    //Initialize angle solver
    AngleSolverParam angleParam;
    angleParam.readFile(9);
    _solverPtr->init(angleParam);
    _solverPtr->setResolution(_videoCapturePtr->getResolution());


    //Initialize armor detector
    ArmorParam armorParam;
    _armorDetectorPtr->init(armorParam);
    _armorDetectorPtr->setEnemyColor(self_color == rm::BLUE ? rm::RED : rm::BLUE);
}

void updateFeeling(list<double>& feeling, const double duration)
{
	for(auto it = feeling.begin(); it != feeling.end(); it++)
	{
		if(*it < 10)
		{
			feeling.erase(it);
		}
		else
		{
			*it *= exp(0.1*duration);
		}
	}
}

void ImgProdCons::sense()
{
//	chrono::time_point<chrono::steady_clock> lastTime;
//	list<double> worrys0, worrys1, pains;
//	double totalWorry0, totalWorry1, totalPain;
//	const double TAU = 0.1;
//	const double M = 10;
//	int remainHp = 3000;

    /* Loop for sensing */
    for(;;)
    {
        FeedBackData feedBackData;

        /* TODO: Handel exceptions when socket suddenly being plugged out. */
        if(_serialPtr->tryFeedBack(feedBackData, chrono::milliseconds(20)) == Serial::OJBK)
        {
//			const auto nowTime = chrono::high_resolution_clock::now();
//			const auto duration = (static_cast<chrono::duration<double, std::milli>>(nowTime - lastTime)).count();

//			/*	Update historic worrys and pain */
//			updateFeeling(worrys0, duration);
//			updateFeeling(worrys1, duration);
//			updateFeeling(pains, duration);

			//TODO: add other states
            _task = feedBackData.task_mode;

//			auto pain = remainHp - feedBackData.remain_hp;
//			remainHp = feedBackData.remain_hp;
//			if(feedBackData.shot_armor == 0)
//			{
//				worrys0.push_back(pain);
//				pains.push_back(pain);
//			}
//			else if(feedBackData.shot_armor == 1)
//			{
//				worrys1.push_back(pain);
//				pains.push_back(pain);
//			}

//			totalWorry0 = accumulate()


//			lastTime = nowTime;
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
    /* Variables for serial communication*/
    ControlData controlData;

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
        if(_serialPtr->getErrorCode() == Serial::SYSTEM_ERROR || !_videoCapturePtr->isOpened())
        {
            this_thread::sleep_for(chrono::seconds(3));
        }
//        switch(_task)
//        {
//        case Serial::NO_TASK:
//            cout<< "manual" <<endl;
//            break;
//        case Serial::SMALL_BUFF:
//            cout<<"small buff"<<endl;
//            break;
//        case Serial::BIG_BUFF:
//            cout<<"big buff"<<endl;
//            break;
//        case Serial::AUTO_SHOOT:
//            cout<<"auto shoot"<<endl;
//            break;
//        default:
//            cout<<"unknown mode"<<endl;
//            break;
//        }


        // comment this part when bebugging without STM upper computer

//        if(_task != Serial::AUTO_SHOOT)
//        {
////            cout << "waiting for command." << endl;
//            continue;
//        }

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

                controlData.frame_seq   = frame.seq;
                controlData.shoot_mode  = Serial::BURST_FIRE | Serial::HIGH_SPEED;
                controlData.pitch_dev   = targetAngle[1];
                controlData.yaw_dev     = targetAngle[0];
//                controlData.speed_on_rail = 0;

#ifdef SENTRY
                controlData.gimbal_mode = Serial::SERVO_MODE;
#endif
                if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
                {
                    cout<<"not sent"<<endl;
                }

                cout << "Deviation: " << targetAngle << endl;
            }
        }

#ifdef SENTRY
        else
        {
            if(_serialPtr->tryControl(controlData, chrono::milliseconds(3)) != Serial::OJBK)
            {
                cout<<"not sent"<<endl;
            }

            controlData.gimbal_mode = Serial::PATROL_AROUND;
        }
#endif

            cout << "Deviation: " << targetAngle << endl;
        }
        auto t2 = chrono::high_resolution_clock::now();
        cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
        cout << endl;
        t1 = t2;


#ifdef DEBUG
        _armorDetectorPtr->showDebugImg();
        if(waitKey(1) == 'q')
        {
            return;
        }
#endif

    }
}
