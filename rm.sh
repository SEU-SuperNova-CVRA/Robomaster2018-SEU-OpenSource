#!/bin/bash
# add the darknet.so into the directory of qt build file
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nvidia/Robomaster/Robomaster2018/Darknet
/home/nvidia/Robomaster/build-Robomaster2018-Desktop-Release/Robomaster2018 >> /home/nvidia/runlog.log
