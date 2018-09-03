#pragma once

#include<opencv2/opencv.hpp>
#include<chrono>
#include<mutex>
#include<memory>

#include"./Driver/RMVideoCapture.hpp"
#include"./Serials/Serial.h"
#include"./Pose/AngleSolver.hpp"
#include"./Armor/ArmorDetector.h"
#include"./Rune/Rune.h"

namespace rm
{
struct Frame
{
	cv::Mat img;
    size_t seq;         //count from 1
    double timeStamp;	//time in ms, from initialization to now
};

/*
 * @Brief:  A simple circular buffer designed only for buffering captured images.
 *          A new frame will cover the oldest one. Every frame has its own mutex
 *          and the mutex is locked when the frame is being updated or being achieved
 *          to ensure thread safety.
 */
class FrameBuffer
{
public:
    FrameBuffer(size_t size);

    ~FrameBuffer() = default;

    bool push(const Frame& frame);

    bool getLatest(Frame& frame);

private:
    std::vector<Frame> _frames;
    std::vector<std::timed_mutex> _mutexs;

    size_t _tailIdx;
    size_t _headIdx;

    double _lastGetTimeStamp;
};

/*
* @Brief:   This class aims at separating reading(producing) images and consuming(using)
*           images into different threads. New images read from the camera are stored
*           into a circular queue. New image will replace the oldest one.
*/
class ImgProdCons
{
public:
    ImgProdCons();
    ~ImgProdCons() {}

    /*
     * @Brief: Initialize all the modules
     */
	void init();

    /*
     * @Brief: Receive self state from the serail port, update task mode if commanded
     */
    void sense();

	/*
    * @Brief: keep reading image from the camera into the buffer
	*/
	void produce();

	/*
    * @Brief: run tasks
	*/
	void consume();

private:
    /*
    * To prevent camera from dying!
    */
    static bool _quit_flag;
    static void signal_handler(int);
    void init_signals(void);

    /* Camera */
    std::unique_ptr<RMVideoCapture> _videoCapturePtr;
    FrameBuffer _buffer;

    /* Serial */
    std::unique_ptr<Serial> _serialPtr;

    /* Angle solver */
    std::unique_ptr<AngleSolver> _solverPtr;

    /* Armor detector */
    std::unique_ptr<ArmorDetector> _armorDetectorPtr;

    /*Rune detector*/
    std::unique_ptr<RuneDetector> _runeDetectorPtr;

    /* @See: 'Serial::TaskMode' */
    volatile uint8_t _task;

	void updateFeelings();
};

}
