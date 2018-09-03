#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>

#include<chrono>

#include "Serial.h"
#include "./General/General.h"

using namespace std;
using namespace rm;

int main()
{
    rm::Serial serial(true);
    serial.openPort();

    rm::ControlData controlData;
    rm::FeedBackData feedBackData;

    int self_color;
    if(serial.setup(self_color))
    {
        cout << "I am " << (self_color == rm::BLUE ? "blue" : "red") << "." << endl;
    }

    uint8_t recordSeq1, recordSeq2;

//    for(;;)
    {
        //auto t1 = std::chrono::high_resolution_clock::now();
        serial.record(recordSeq1);

        sleep(1);

        serial.record(recordSeq2);

        sleep(1);

        controlData.Seq = recordSeq1;
        controlData.ShootMode = BurstFire | HighBulletSpeed;
        controlData.TargetPitchAngle = 10;
        controlData.TargetYawAngle = 20;
        controlData.TargetSpeedOnRail = 3;
        controlData.SentryGimbalMode = ServoMode;
        serial.control(controlData);

        sleep(1);

        serial.feedBack(feedBackData);
        cout<<static_cast<int>(feedBackData.ShootSpeed)<<endl;
        cout<<static_cast<int>(feedBackData.RemainHP)<<endl;

        //auto t2 = std::chrono::high_resolution_clock::now();



        //cout<<"time cost: "<<(chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)).count() << "ms" << endl<<endl;

        sleep(1);
    }


	return 0;
}

