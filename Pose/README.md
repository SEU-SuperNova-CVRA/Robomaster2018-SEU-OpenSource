# Pose
## 1. Algorithm Description
### 1.1 Overview
 位姿检测是承前启后的一部分，它接受装甲板检测的位置信息或者大小符检测的九宫格位置信息，解算距离信息和角度信息。其中距离信息目前用于重力补偿，角度信息主要是解算光心与目标中心偏差角度，发送给下位机，从而实现精确瞄准。
 
解算角度最原始的办法是获得摄像头的视角，平均分割图像，利用目标点到图像中心的距离与屏幕长/宽相比，在乘以相应方向的视角，可以得到偏差角度。然而，由于摄像头的制造有一定误差，摄像头收到的图像又有一定的畸变，这种解算方式会遭遇不可排除的误差，也不可能获得距离，这对精确打击、重力补偿是致命的。所以，我们需要更为精确的办法。

### 1.2 运行思路
1.从其他类获得一个点或者四个点的位置坐标。

2.进行位姿解算。这一步会根据输入参数的不同进行两种解算模式：单点解算和P4P解算。

3.进行所需的补偿。

4.给出偏差角。

### 1.3 运行流程
1.标定摄像头。（预完成）

2.初始化AnglesolverParam类。用它初始化Anglesolver类，设定其他初始参数，包含设定画面大小，以及一些关于解算模式的参数。如果没有特殊要求，这部分参数不需要改变，在继续开发过程中，这些参数还有其他用途。这涉及到一些还没有完成的功能和正在测试的功能的开启与否，不改变则不开启。初始化predictor类。（这个类是用来做预瞄准的，已经实测实现了一定预瞄效果，待继续实测，未同步，邀请友校同学共同开发）

3.调用Anglesolver.setTarget。也就是设置目标，会根据给的目标是一点还是四点进入不同模式。（如果启用预瞄准，是在这一步前调用Predictor.set和Predictor.predict，把predict返回的预测位置发给Anglesolver.setTarget。不用则忽略括号中内容，这一部分的test实例待同步）

4.调用Anglesolver.solve。这一部分是核心。它会根据模式不同来解算。解算结果自动存在Anglesolver类中的对应变量。

5.调用补偿函数，调用Anglesolver.getXXXXXX，返回解算结果。

### 1.4 PNP解法

通过将视觉获取的N个点的坐标，和这些点在已知的三位模型中的坐标对比，接触目标物体相对于摄像头的位姿。这个位姿是用一个旋转矩阵和一个位移矩阵表达的。Opencv中有该函数。
下图为cv：：solvePnP的函数原型。

![image](https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource/blob/master/Img/pnp.png)

### 1.5 补偿
由于重力影响，需要在y轴方向进行重力补偿，这一部分在const cv::Vec2f AngleSolver::getCompensateAngle()里实现。

 由于枪口与摄像头光心的偏置，子弹发射方向与Z轴有偏差，安装误差以及一些难以预计的误差，进行弹道补偿很有必要，且在近似情况下，弹道补偿可以把重力补偿值包含。可以通过测量期望落点和实际落点之间的偏差补偿。
 
 
 
## 2.API
### rm::AngleSolver.init(const AngleSolverParam& AngleSolverParam)
initialize the `AngleSolver` instance
#### Args:
initialized instance of `rm::AngleSolverParam`
#### Returns:
`void`


---
### rm::AngleSolver.setTarget(const std::vector<cv::Point2f> objectPoints, int objectType) /setTarget(const cv::Point2f centerPoint, int objectType)
set the center point or corners of armor, or the center of buff as target
### Args:
corner points(`vector`) or center point(`cv::Point2f`), and what the object is(`int`).
### Returns:
`void`


---
### rm::AngleSolver.solve()
solve the pose problem
### Args:
None
### Returns:
`void`


---
### rm::AngleSlover.setResolution(const cv::Size2i& image_resolution)
tell anglesolver the resolution of used image
### Args:
resolution of used image(`cv::Size2i`)
### Returns:
`void`


---
### rm::AngleSlover.getCompensateAngle()
get the error angle after compensated by the consider of gravity
### Args:
None
### Returns:
const `cv::Vec2f` errAngle

---
### rm::AngleSlover.getAngle()
get the error angle 
### Args:
None
### Returns:
const `cv::Vec2f` errAngle

---
### rm::AngleSlover.getDistance()
get the distance between the camera and the target
### Args:
None
### Returns:
const `double` distance


---
### rm::AngleSolver.setBulletSpeed(int bulletSpeed)
set the speed of bullet
### Args:
`int` bullet speed
### Returns:
void
