# Robomaster2018-SEU-OpenSource
This is a project for Robomaster 2018 from Southeast University which includes complete process to finish the auto-shoot & rune detection tasks. Armor detection, rune detection, angle solving algorithm & drivers, serial communication are all included.

This is basically the code we use during the competition.

## 1.Requirements
### platform:
>1. Jetson TX2
>2. ubuntu16.04
### environment
>1. QT5
>2. OpenCV3.4.0(Opencv4Tegra)

## 2.Project framework

**Armor**,**Rune**&**Pose**: three core algorithm are implemented here. You can read the document under these three directory.

**Darknet**: the deeplearning library we need to run this project, you need to compile it first.

**Driver**: driver for camera.

**General**: general resource for all program.

**Serial**: serial communication protocol with STM32.

**Main**: entry of the program.

**Img**: resource files for document


## 3.Configuration
### 1. Clone the project
Clone the project to the directory you perfer.
If you are not sure, `/home/usrname/` is just OK.
### 2. Compile the darknet library
The darknet library is the required dependency lib to implement the deep learning algorithm in Rune Detection. We made a little changes to the original library to fit our need. Anyway, be sure to compile it first, or the project can't run successfully.

*NOTICE:Unless you need to test only a part of the project, like Armor, you can rewrite the project dependency and run it independently*

**How to compile in terminal**

1. change directory the `./Darknet`
2. type `make #-j4 optional, it depends on your machine`

A README from the official darknet project is also under the `./Darknet`. You can ignore that unless you're interested in the darknet.
### 3. Choose the main file
Open the .pro file, and choose the main.cpp file according to the instruction(comment) in .pro by commenting & uncommenting several lines of codes.

If you want to design your own test, you can write your own main file and add it to the project.
### 4. Others
change the absolute path of .xml(under ./Pose directory)

If you meet any other errors, remeber to check the directory in the project(especially some .xml files) first.

## 4.How To Run
After successfully compile the whole project in QT, you shall check the executable file `Robomaster2018-SEU-OpenSource` produced by QT(under the `./Release` or `./Debug` directory). Excute it, and you can see the result.


**Auto Startup**

Also, we add the script `rm.sh` for the Startup Application.

Be sure to change the content (Directory of executable file and so on) for your own machine. Also, DON'T forget to add the darknet.so(produced after compiling this lib) to the same directory of executable file

Add this script to the **Startup Applications** and everything is okay now!


## 5.License

[MIT License](https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource/blob/master/LICENSE)

## 6.Contributors

BINYAN HU, CHENGHAO SU, BOWEN SUN, HANG ZHOU, SHU SHI, JIHAN ZHANG, YIRAN FANG.
