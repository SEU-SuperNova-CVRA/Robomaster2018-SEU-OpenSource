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

#include "RMVideoCapture.hpp"
#include "linux/videodev2.h"
#include <libv4l2.h>
#include <errno.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <iostream>
#include <string>
#include <chrono>

using namespace std;

int RMVideoCapture::xioctl(int fd, int request, void *arg)
{
	int r;
	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);
	return r;
}

RMVideoCapture::RMVideoCapture()
{
    _bufferIdx = 0;
    _frameCount = 0;
    _width = 0;
    _height = 0;
    _grabbed_left = false;
}

RMVideoCapture::RMVideoCapture(const int id, int buffer_size)
{
    _videoPath = ("/dev/video" + to_string(id));
    _buffer_size = buffer_size;

    _buffers = new MapBuffer[_buffer_size];

    _bufferIdx = 0;
    _frameCount = 0;
    _width = 0;
    _height = 0;

    _camera_fd = v4l2_open(_videoPath.c_str(), O_RDWR);
    //_camera_fd = v4l2_open(_videoPath, O_RDWR | O_NONBLOCK);
    if(_camera_fd == -1)
	{
        _is_opened = false;
        cout << "Opening camera " + to_string(id) + " failed." << endl;
	}
    else
    {
        _is_opened = true;
        cout << "Opening camera " + to_string(id) + " succeeded." << endl;
    }
}

RMVideoCapture::~RMVideoCapture()
{
    v4l2_close(_camera_fd);
    delete [] _buffers;
}

bool RMVideoCapture::open(const int id, int size_buffer)
{
    _videoPath = "/dev/video" + to_string(id);
    _buffer_size = size_buffer;
    _buffers = new MapBuffer[_buffer_size];

    _camera_fd = v4l2_open(_videoPath.c_str(), O_RDWR);
    //_camera_fd = v4l2_open(_videoPath, O_RDWR | O_NONBLOCK);
    if(_camera_fd == -1)
    {
        _is_opened = false;
        cout << "Opening camera " + to_string(id) + " failed." << endl;
    }
    else
    {
        _is_opened = true;
        cout << "Opening camera " + to_string(id) + " succeeded." << endl;
    }

    return _is_opened;
}

bool RMVideoCapture::isOpened()const
{
    return _is_opened;
}

//void RMVideoCapture::setBufferSize(size_t bsize)
//{
//    if (_buffer_size != bsize)
//	{
//        _buffer_size = bsize;
//        delete [] _buffers;
//        _buffers = new MapBuffer[_buffer_size];
//	}
//}

bool RMVideoCapture::setVideoFormat(size_t width, size_t height, bool mjpg)
{	 
    if (_width == width && _height == height)
	{
		return true;
	}
    _width = width;
    _height = height;
    _frameCount = 0;
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	if (mjpg == true)
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	else
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (-1 == xioctl(_camera_fd, VIDIOC_S_FMT, &fmt))
	{
        cout << "Setting Pixel Format failed" <<endl;
		return false;
	}
	return true;
}

bool RMVideoCapture::changeVideoFormat(int width, int height, bool mjpg){
	closeStream();
	restartCapture();
	setVideoFormat(width, height, mjpg);
	startStream();
	return true;
}

bool RMVideoCapture::refreshVideoFormat()
{
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(_camera_fd, VIDIOC_G_FMT, &fmt))
	{
		perror("Querying Pixel Format\n");
		return false;
	}
    _width = fmt.fmt.pix.width;
    _height = fmt.fmt.pix.height;
    _format = fmt.fmt.pix.pixelformat;
	return true;
}

bool RMVideoCapture::setFPS(int fps)
{
    struct v4l2_streamparm stream_param;
    memset(&stream_param, 0, sizeof(stream_param));
	stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_param.parm.capture.timeperframe.denominator = fps;
	stream_param.parm.capture.timeperframe.numerator = 1;

    if (-1 == xioctl(_camera_fd, VIDIOC_S_PARM, &stream_param))
	{
		printf("Setting Frame Rate failed.\n");
		return false;
	}
	return true;
}

bool RMVideoCapture::setExposureTime(int t)
{
	if (t == 0)
	{
		struct v4l2_control control_s;
		control_s.id = V4L2_CID_EXPOSURE_AUTO;
		control_s.value = V4L2_EXPOSURE_AUTO;
        if( xioctl(_camera_fd, VIDIOC_S_CTRL, &control_s) < 0)
		{
			printf("Set Auto Exposure error\n");
			return false;
		}
	}
	else 
	{
/*
		struct v4l2_control control_s;
		control_s.id = V4L2_CID_EXPOSURE_AUTO;
		control_s.value = V4L2_EXPOSURE_MANUAL;
        if( xioctl(_camera_fd, VIDIOC_S_CTRL, &control_s) < 0)
		{
			printf("Close MANUAL Exposure error\n");
			return false;
		}

		control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
		control_s.value = t;
        if( xioctl(_camera_fd, VIDIOC_S_CTRL, &control_s) < 0)
		{
			printf("Set Exposure Time error\n");
			return false;
		}
*/
		{
			// not sure why, but v4l2_set_control() does not work for
			// V4L2_CID_EXPOSURE_AUTO...
			struct v4l2_control c;
			c.id = V4L2_CID_EXPOSURE_AUTO;
			c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
            if (v4l2_ioctl(_camera_fd, VIDIOC_S_CTRL, &c) != 0)
			{
				cout << "Failed to set... " << strerror(errno) << endl;
			}
			cout << "exposure: " << t << endl;
            v4l2_set_control(_camera_fd, V4L2_CID_EXPOSURE_ABSOLUTE, t*6);
		}
	}

	return true;
}

bool RMVideoCapture::initMMap()
{
    struct v4l2_requestbuffers bufrequest{};
	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = _buffer_size;

    if(ioctl(_camera_fd, VIDIOC_REQBUFS, &bufrequest) < 0)
	{
		printf("Requesting buffer failed.\n");
		return false;
	}

    for(size_t i = 0; i < _buffer_size; ++i)
	{
        struct v4l2_buffer bufferinfo;
        memset(&bufferinfo, 0, sizeof(bufferinfo));
		bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		bufferinfo.memory = V4L2_MEMORY_MMAP;
		bufferinfo.index = i; /* Queueing buffer index 0. */

		// mmap for buffers
        if(ioctl(_camera_fd, VIDIOC_QUERYBUF, &bufferinfo) < 0)
		{
			perror("Buffer map failed.\n");
			return false;
		}

        _buffers[i].ptr = mmap(
			NULL,
			bufferinfo.length,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
            _camera_fd,
			bufferinfo.m.offset);
        _buffers[i].length = bufferinfo.length;

        if(_buffers[i].ptr == MAP_FAILED)
		{
			perror("Buffer map failed.\n");
			return false;
		}

        memset(_buffers[i].ptr, 0, bufferinfo.length);

		// Put the buffer in the incoming queue.
		memset(&bufferinfo, 0, sizeof(bufferinfo));
		bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		bufferinfo.memory = V4L2_MEMORY_MMAP;
		bufferinfo.index = i;
        if(ioctl(_camera_fd, VIDIOC_QBUF, &bufferinfo) < 0)
		{
			perror("Querying buffer ");
			return false;
		}
	}

	return true;
}

bool RMVideoCapture::startStream()
{
    _frameCount = 0;
	refreshVideoFormat();
	if(initMMap() == false) return false;

	__u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(_camera_fd, VIDIOC_STREAMON, &type) < 0)
	{
		perror("Starting stream failed.\n");
		return false;
	}
	return true;
}

bool RMVideoCapture::closeStream()
{
    _frameCount = 0;
    _bufferIdx = 0;
	__u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(_camera_fd, VIDIOC_STREAMOFF, &type) < 0)
	{
		perror("VIDIOC_STREAMOFF");
		return false;
	}
    for(size_t i = 0; i < _buffer_size; ++i)
	{
        munmap(_buffers[i].ptr, _buffers[i].length);
	}
	return true;
}

void RMVideoCapture::restartCapture()
{
    close(_camera_fd);
    _camera_fd = v4l2_open(_videoPath.c_str(), O_RDWR);
    _bufferIdx = 0;
    _frameCount = 0;
}

void RMVideoCapture::cvtRaw2Mat(void* data, cv::Mat& image)
{
    if (_format == V4L2_PIX_FMT_MJPEG)
	{
		//OpenCV method, 60fps
        cv::Mat mjpg(_height, _width, CV_8UC3, data);
        image = cv::imdecode(mjpg, CV_LOAD_IMAGE_COLOR);

		//libjpeg-turbo method, 30fps
		//image = Jpeg2Mat((uint8_t*)data,length);
	}
    else if(_format == V4L2_PIX_FMT_YUYV)
	{
        cv::Mat yuyv(_height, _width, CV_8UC2, data);
		cv::cvtColor(yuyv, image, CV_YUV2BGR_YUYV);
	}
}

RMVideoCapture & RMVideoCapture::operator >> (cv::Mat& image)
{
//	auto t1 = std::chrono::high_resolution_clock::now();

    if(grab())
    {
        //	auto t2 = chrono::high_resolution_clock::now();
        //    cout << "Capturing cost: " << (chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)).count() << "ms" << std::endl;

        retrieve(image);

        //    auto t3 = chrono::high_resolution_clock::now();
        //    cout << "Decoding cost: "  << (chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)).count() << "ms" << std::endl;
    }
	return *this;
}

bool RMVideoCapture::grab()
{
    if(_grabbed_left)
    {
        // Put buffed image to the end
        struct v4l2_buffer bufferinfo;
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = _bufferIdx;
        if(ioctl(_camera_fd, VIDIOC_QBUF, &bufferinfo) < 0)
        {
            perror("VIDIOC_QBUF Error");
            return false;
        }
        else
        {
            ++_bufferIdx;
            if(_bufferIdx >= _buffer_size)
            {
                _bufferIdx -= _buffer_size;
            }
        }
    }

    struct v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = _bufferIdx;
    if(ioctl(_camera_fd, VIDIOC_DQBUF, &bufferinfo) < 0)
    {
        perror("VIDIOC_DQBUF Error");
        return false;
    }
    else
    {
        _grabbed_left = true;
        return true;
    }
}

bool RMVideoCapture::retrieve(cv::Mat& image)
{
    cvtRaw2Mat(_buffers[_bufferIdx].ptr, image);

    // Put buffed image to the end
    struct v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = _bufferIdx;
    if(ioctl(_camera_fd, VIDIOC_QBUF, &bufferinfo) < 0)
    {
        perror("VIDIOC_QBUF Error");
//		exit(1);
        return false;
    }
    else
    {
        ++_bufferIdx;
        if(_bufferIdx >= _buffer_size)
        {
            _bufferIdx -= _buffer_size;
        }

        ++_frameCount;

        _grabbed_left = false;
        return true;
    }
}

cv::Size RMVideoCapture::getResolution()
{
    return cv::Size(_width, _height);
}

void RMVideoCapture::info()
{
	struct v4l2_capability caps = {};
    if (-1 == xioctl(_camera_fd, VIDIOC_QUERYCAP, &caps))
	{
			perror("Querying Capabilities\n");
			return;
	}

	printf( "Driver Caps:\n"
			"  Driver: \"%s\"\n"
			"  Card: \"%s\"\n"
			"  Bus: \"%s\"\n"
			"  Version: %d.%d\n"
			"  Capabilities: %08x\n",
			caps.driver,
			caps.card,
			caps.bus_info,
			(caps.version>>16)&&0xff,
			(caps.version>>24)&&0xff,
			caps.capabilities);


    struct v4l2_cropcap cropcap;
    memset(&cropcap, 0, sizeof(cropcap));
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl (_camera_fd, VIDIOC_CROPCAP, &cropcap))
	{
			perror("Querying Cropping Capabilities\n");
			return;
	}

	printf( "Camera Cropping:\n"
			"  Bounds: %dx%d+%d+%d\n"
			"  Default: %dx%d+%d+%d\n"
			"  Aspect: %d/%d\n",
			cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
			cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
			cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    struct v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	char fourcc[5] = {0};
	char c, e;
	printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(_camera_fd, VIDIOC_ENUM_FMT, &fmtdesc))
	{
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
	}

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(_camera_fd, VIDIOC_G_FMT, &fmt))
	{
		perror("Querying Pixel Format\n");
		return;
	}
	strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
	printf( "Selected Camera Mode:\n"
			"  Width: %d\n"
			"  Height: %d\n"
			"  PixFmt: %s\n"
			"  Field: %d\n",
			fmt.fmt.pix.width,
			fmt.fmt.pix.height,
			fourcc,
			fmt.fmt.pix.field);

    struct v4l2_streamparm streamparm;
    memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(_camera_fd, VIDIOC_G_PARM, &streamparm))
	{
		perror("Querying Frame Rate\n");
		return;
	}
	printf( "Frame Rate:  %f\n====================\n",
            (float)streamparm.parm.capture.timeperframe.denominator /
            (float)streamparm.parm.capture.timeperframe.numerator);
}
