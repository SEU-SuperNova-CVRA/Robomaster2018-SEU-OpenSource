# Armor Detection
## 1. Algorithm Description
### 1.1 Overview
基于灯条匹配的装甲板识别采用约束集的方法的总体思想是:先检测感兴趣颜色的装甲板灯条,再通过对灯条的匹配来得到装甲板。总体分为如下几个步骤:
>分割提取灯条 —> 筛选灯条 —> 灯条匹配 —> 装甲板图案识别

在整个识别流程中，除了算法上的图像识别，还有需要更加高级的决策层
目前我们实现了两个简单原型：　目标跟踪机制以及目标选择机制

### 1.2 识别算法
#### 1.2.1 灯柱提取
灯柱提取是整个检测过程中最基本，但同时也是最难的问题之一。在整个调试过程中，由于相机，光线条件，距离等条件的不同，如果没有工业级别的相机，想要实现完全鲁棒的算法具有很大的挑战性。在经历过很多实验与尝试后，最后决定采用**实用**导向的思路：不追求鲁棒性，追求在特定的条件下能够实现足够的准确性。
我们最后采用的方案是：
>３m内＋低曝光＋亮度通道阈值化＋颜色判断

经过测试，准确率完全达标
#### 1.2.2 灯柱筛选
#### 约束:
>1. 轮廓面积大于一定值
>2. 轮廓的长宽比大于一定值
>3. 轮廓的凸度(Solidity)大于一定值
>4. 轮廓内的颜色是目标颜色

其中,凸度(Solidity)定义为:

![image](https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource/blob/master/Img/CodeCogsEqn.png)  
S_c : 灯柱轮廓的面积  
S_f : 外包椭圆的面积  
判断轮廓内部颜色采用遍历轮廓内部像素点,并统计颜色分布的方法。对矩形内的三通道分别取均值,若B-G > threshold则初步认为这可能是一个蓝色灯条,反之则为红色。最终对符合条件的灯条构建描述。
#### 1.2.3 灯条匹配
利用约束集对灯条进行两两匹配,筛选不可能成为装甲板的组合,同时为可能成为装甲板的区域评分。
#### 约束:
>1. 两个灯条近似平行
>2. 两个灯条的高度大小近似相似
>3. 中心点 y 坐标差距不大
>4. 装甲板区域的长宽比在一定范围内

同一灯条在左右两个方向上可能分别有多个满足约束的匹配,由于每个灯条最终只有可能与另外一个灯条匹配称为装甲板,所以在这一步可以采用“非极大抑制”的思想,只选在满足约束的匹配中距离最近的匹配。
效果图如下
![image](https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource/blob/master/Img/nms.jpg)
最终为匹配的灯条对,即装甲板构建描述(ArmorDescriptor)。
#### 1.2.4 装甲板图案识别
对于已经匹配的灯条，通过透视变换提取装甲板图案的正视图。

对于图案的识别我们简单采用了SVM作为分类器。正确率在可接受范围内。
### 1.3 决策机制
#### 1.3.1 目标选择机制
对所有的候选装甲板目标进行三个方面的评分
>1. 装甲板的大小
>2. 装甲板距离视野中心的距离
>3. 装甲板的视野正对程度

在所有通过前面约束的装甲板目标中，选择得分最高的装甲板作为最终打击目标。
#### 1.3.2 跟踪机制
对上次识别到的目标，下一次识别以此区域为基础，只处理附近的roi。

为防止在跟踪过程中忽略了视跟踪视野外出现潜在评分最高的目标（最大威胁目标），需要在固定帧数后对全局进行一次目标再选择

### 1.4 问题与缺陷
1. 相机条件限制，导致目前的算法根本无法处理高速目标（哨兵无法实现高速搜索目标）。
2. 依靠亮度筛选灯条会过于依赖调参。
3. 跟踪机制不健全,没有考虑到相对运动造成roi的偏移。
### 1.5 TO DO
1. 尝试使用更加高级以及更具鲁棒性的决策层，给自动瞄准加入灵魂
2. 针对步兵和哨兵两种机器人采用不同的人机交互方式。对于全自动机器人需要全权给予决策权，对于决策的要求更高级更智能。而对于步兵需要设计出更加适应操作手的人机交互方式。
3. 尝试cnn等深度学习方法进行分类／实现全局直接搜索装甲板图案
## 2.API
### rm::ArmorDetector.init(const ArmorParam& armorParam)
initialize the `ArmorDetector` instance
#### Args:
initialized instance of `rm::ArmorParam`
#### Returns:
`void`

---
### rm::ArmorDetector.setEnemyColor(int enemy_color)
set the color of enemy target.
#### Args:
flag value of enemy color  `int`, 0 stands for blue, 1 stands for red
#### Returns:
`void`
#### See Also:
`./General/General.h`

---
### rm::ArmorDetector.loadImg(const Mat& srcImg)
receive the source image from the camera.
#### Args:
source image `Mat srcImg`.
#### Returns:
`void`

---
### rm::ArmorDetector.Detect()
main function to detect the armor.
#### Args:
`None`
#### Returns:
flag value of detection result `int`.
```
enum ArmorFlag
{
  ARMOR_NO = 0,		// not found
  ARMOR_LOST = 1,		// lose tracking
  ARMOR_GLOBAL = 2,	// armor found globally
  ARMOR_LOCAL = 3		// armor found locally(in tracking mode)
};
```

---
### rm::ArmorDetector.getArmorVertex()
return the location property of target for further process (angle solving mainly)
#### Args:
`None`
#### Returns:
four corner points `vector<cv::Point2f>` of target armor.

---
### rm::ArmorDetector.getArmorType()
return the type property of target for further process (angle solving mainly).
#### Args:
`None`
#### Returns:
flag value of armor type `int`, 0 stands for unknown (fail to detect), 1 stands for small armor, 2 stands for big armor.
#### See Also:
`./General/General.h`
