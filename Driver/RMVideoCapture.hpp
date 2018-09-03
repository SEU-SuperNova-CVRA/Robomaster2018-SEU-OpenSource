/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#pragma once
#include<opencv2/opencv.hpp>

class RMVideoCapture
{
public:
    RMVideoCapture();
    RMVideoCapture(const int id, int buffer_size = 2);
	~RMVideoCapture();

    bool open(const int id, int size_buffer = 2);
    bool isOpened() const;

//    void setBufferSize(size_t bsize);
    bool setVideoFormat(size_t width, size_t height, bool mjpg = 1);
    bool setExposureTime(int t = 0);
	bool changeVideoFormat(int width, int height, bool mjpg = 1);
	bool setFPS(int fps);

	bool startStream();
	bool closeStream();
	void restartCapture();

	RMVideoCapture& operator >> (cv::Mat & image);

    bool read(cv::Mat& image);

    /*
     * @Brief:  Grabs the next frame from video file or capturing device.
     * @Return: 'true' if success
     */
    bool grab();

    /*
     * @Brief:  Decodes and returns the grabbed video frame.
     * @Output: img:    the video frame is returned here. If no frames has been grabbed
     *                  the image remains unchanged.
     */
    bool retrieve(cv::Mat& image);

	cv::Size getResolution();
	int getFrameCount()
	{
        return _frameCount;
	}

	void info();


private:
	struct MapBuffer
	{
		void * ptr;
		unsigned int length;
	};

    unsigned int _width;
    unsigned int _height;
    unsigned int _format;

    int _camera_fd;
    bool _is_opened;

    unsigned int _buffer_size;
    unsigned int _bufferIdx;
    unsigned int _frameCount;
    MapBuffer* _buffers;
    std::string _videoPath;

    bool _grabbed_left;


    //void cvtRaw2Mat(void * data,unsigned int length, cv::Mat & image);
    void cvtRaw2Mat(void * data, cv::Mat & image);
    bool refreshVideoFormat();
    bool initMMap();
    int xioctl(int fd, int request, void *arg);
};

