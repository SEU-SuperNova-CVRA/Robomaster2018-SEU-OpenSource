/**************************************************************
MIT License
Copyright (c) 2018 SEU-SuperNova-CVRA
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Authors:    BinYan Hu
**************************************************************/
#pragma once

#include<iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <mutex>
#include <chrono>
#include<thread>

/* Serial frame's EOF may conflict with somewhere else's EOF */
#undef EOF

namespace rm
{

struct ControlData
{
    uint8_t    frame_seq;           //Corresponding to the record
    uint16_t   shoot_mode;          //高八位发射方式 低八位发射速度等级  (0xFFFF-记录当前角度  0xEEEE-红方  0xDDDD-蓝方  0xCCCC-通信建立  0xBBBB-Jetson为了接收数据发送帧)
    float      pitch_dev;           //Pitch目标角度
    float      yaw_dev;             //Yaw目标角度
    /*  哨兵专用   */
    int16_t    rail_speed;          //目标轨道速度（哨兵用）
    uint8_t    gimbal_mode;         //哨兵云台攻击模式
};

struct FeedBackData
{
    uint8_t    task_mode;           //所需控制模式
    uint8_t    bullet_speed;        //射速
    /*  哨兵专用   */
    uint8_t    rail_pos;            //所处轨道标号
    uint8_t    shot_armor;          //被打击装甲板标识
    uint16_t   remain_HP;           //剩余血量
};


/*
 * @Brief:  Serial communication protocol and inplement
 */
class Serial
{
public:
    /*
     * @Brief: 比赛红蓝方
     */
    enum TeamName
    {
        BLUE_TEAM       =   (uint16_t)0xDDDD,
        RED_TEAM        =   (uint16_t)0xEEEE
    };

    /*
     * @Brief: control frame mode
     */
    enum ControlMode
    {
        SET_UP          =   (uint16_t)0xCCCC,
        RECORD_ANGLE    =   (uint16_t)0xFFFF,
        REQUEST_TRANS   =   (uint16_t)0xBBBB
    };

    /*
     * @Brief: 发射方式
     */
    enum ShootMode
    {
        NO_FIRE         =   (uint16_t)(0x00<<8),//不发射
        SINGLE_FIRE     =   (uint16_t)(0x01<<8),//点射
        BURST_FIRE      =   (uint16_t)(0x02<<8) //连发
    };

    /*
     * @Brief: 发射速度
     */
    enum BulletSpeed
    {
        HIGH_SPEED      =   (uint16_t)(0x01),   //高速
        LOW_SPEED       =   (uint16_t)(0x02)    //低速
    };

    /*
     * @Breif:所需控制模式
     */
    enum TaskMode
    {
        NO_TASK         =   (uint8_t)(0x00),    //手动控制
        SMALL_BUFF      =   (uint8_t)(0x01),    //小符模式
        BIG_BUFF        =   (uint8_t)(0x02),    //大符模式
        AUTO_SHOOT      =   (uint8_t)(0x03)     //自动射击
    };

    /*
     * @Brief: 哨兵云台工作模式
     */
    enum GimbalMode
    {
        PATROL_AROUND   =   (uint8_t)(0x01),    //旋转巡逻
        PATROL_ARMOR_0  =   (uint8_t)(0x02),    //巡逻装甲板0
        PATROL_ARMOR_1  =   (uint8_t)(0x03),    //巡逻装甲板1
        SERVO_MODE      =   (uint8_t)(0x04)     //伺服打击
    };

    /* @Brief:
     *      SYSTEM_ERROR:   System error catched. May be caused by wrong port number,
     *                      fragile connection between Jetson and STM, STM shutting
     *                      down during communicating or the sockets being suddenly
     *                      plugged out.
     *      OJBK:         Everything all right
     *      PORT_OCCUPIED:  Fail to close the serial port
     *      READ_WRITE_ERROR: Fail to write to or read from the port
     *      CORRUPTED_FRAME: Wrong frame format
     *      TIME_OUT:       Receiving time out
     */
    enum ErrorCode
    {
        SYSTEM_ERROR    = 1,
        OJBK            = 0,
        PORT_OCCUPIED   = -1,
        READ_WRITE_ERROR= -2,
        CORRUPTED_FRAME = -3,
        TIME_OUT        = -4
    };

public:
    Serial();
    Serial(const Serial& right) = delete;
    Serial(Serial&& ) = delete;
    ~Serial();


    /*
     * @Brief: Open serial port and config options
     */
    int openPort();

    /*
     * @Brief:  close serial port
     */
    int closePort();

    /*
     * @Brief
     */
    bool isOpened() const;

    /*
     * @Brief: std::cout detailed states and information when en_debug is on.
     */
    void setDebug(const bool en_debug);

    /*
     * @Brief:  Set up communication with STM
     * @Output: self_team: BLUE_TEAM or RED_TEAM
     */
    int setup(int& self_color);

    /*
     * @Brief:  记录当前的角度
     * @Input:  time_duration: try for how long to record
     * @Output: frame_seq: seq of the sent record-frame
     */
    template<typename _Rep, typename _Period>
    int tryRecord(uint8_t& frame_seq, const std::chrono::duration<_Rep, _Period>& time_duration)
    {
        std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
        if(lockGuard.owns_lock())
        {
            record(frame_seq);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return _errorCode;
        }
        else
        {
            return PORT_OCCUPIED;
        }
    }

    /*
     * @Brief:  控制机器人
     * @Input:  time_duration:  try for how long to send control data
     */
    template<typename _Rep, typename _Period>
    int tryControl(const ControlData& controlData, const std::chrono::duration<_Rep, _Period>& time_duration)
    {
        std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
        if(lockGuard.owns_lock())
        {
            control(controlData);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return _errorCode;
        }
        else
        {
            return PORT_OCCUPIED;
        }
    }

    /*
     * @Brief:  从机器人接收信息
     * @Input:  time_duration:  try for how long to receive feedback data
     */
    template<typename _Rep, typename _Period>
    int tryFeedBack(FeedBackData& feedBackData, const std::chrono::duration<_Rep, _Period>& time_duration)
    {
        std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
        if(lockGuard.owns_lock())
        {
            feedBack(feedBackData);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return _errorCode;
        }
        else
        {
            return PORT_OCCUPIED;
        }
    }

    /*
     * @Brief:  DO NOT call this right after the record, control or feedback because
     *          '_errorCode' will not be thread safe after they return, which means
     *          that you may not get the exact error code representing the operation state.
     *          Call this function in the case which does not worry about thread safety.
     */
    int getErrorCode() const;

private:
    /*
     * @Brief: 帧头帧尾
     */
    enum
    {
        JetsonCommSOF = (uint8_t)0x66,
        JetsonCommEOF = (uint8_t)0x88
    };

private:
    /* -1 if serial port not opened */
    int _serialFd;

    /* @See: 'enum ErrorCode' */
    int _errorCode;

    /* 输出详细的记录至控制台 if true */
    bool _en_debug;

    uint8_t _lastRecordSeq;

    std::timed_mutex _mutex;

    /*
     * @Brief: TX2控制战车帧结构体
     */
    struct ControlFrame
    {
        uint8_t  SOF;
        uint8_t  frame_seq;
        uint16_t shoot_mode;
        float    pitch_dev;
        float    yaw_dev;
        int16_t  rail_speed;
        uint8_t  gimbal_mode;
        uint8_t  EOF;
    }_controlFrame;

    /*
     * @Brief: 战车回传数据帧结构体
     */
    struct FeedBackFrame
    {
        uint8_t  SOF;
        uint8_t  frame_seq;
        uint8_t  task_mode;
        uint8_t  bullet_speed;
        uint8_t  rail_pos;
        uint8_t  shot_armor;
        uint16_t remain_HP;
        uint8_t  reserved[11];
        uint8_t  EOF;
    }_feedBackFrame;

//    std::ostream& operator<<(ostream& os, const ControlFrame& cf);
    void print(const ControlFrame& ct);
    void print(const FeedBackFrame& fb);

    int record(uint8_t& frame_seq);
    int control(const ControlData& controlData);
    int feedBack(FeedBackData& feedBackData);

    ControlFrame pack(const ControlData& controlData);
    FeedBackData unpack(const FeedBackFrame& FeedBackData);

    int send();
    int receive();
};

}
