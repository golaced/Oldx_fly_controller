
# 1 OLDX-开源多旋翼开发平台项目  
<div align=center><img width="600" height="130" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/logo.JPG"/></div>
<div align=center><img width="440" height="280" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/fc.jpg"/></div>
  OLDX多旋翼开发平台（OLDX-FC）是由北京理工大学自动化学院所属极客团队开发的一个目前国内最完整的免费开源飞控项目，随着国内开源飞控的逐步发展如匿名、
INF、无名和ACFly飞控的陆续推出，如光流、气压计和GPS等相关算法已经逐步完善，但是相比Pixhawk等国外开源飞控平台的发展和定位仍然太过局限很多团队仍采用
代码拷贝淘宝二次销售的形式导致经济竞争而非技术竞争。OLDX-FC于14年开始对多旋翼飞行器进行研究期间也经历过开源和借鉴的过程，为希望进一步推动国内开源飞控协作开发和
相互学习、相互分享的趋势，团队将该OLDX-FC转化为开源项目，采用自由捐赠的形式继续发展（[捐赠地址](https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.png)。
项目遵循GPL协议，能自由下载项目PCB进行加工使用但请勿作为商业用途，开源所有飞行控制和组合导航源码，可以进行修改和二次开发。<br><br><br>

**项目荣誉**

项目|奖项|年份
-------------|-------------|-------------
IMAV国际微小型无人机大赛|室外赛第3名  室内最佳自动化|2018
中航工业杯|三等奖|2018
IMAV国际微小型无人机大赛|室外赛第5名|2017
中航工业杯|二等奖|2017
中航工业杯|三等奖|2016

**-如果该项目对您有帮助请 Star 我们的项目-**<br>
**-如果您愿意分享对该项目的优化和改进请联系golaced@163.com或加入我们的QQ群，加速开源项目的进度-**<br>

	
# 2 基本功能介绍
  OLDX-FC是一个基于STM32F4系列单片机的多旋翼飞控平台，其采用双处理器、双IMU冗余的设计，飞行控制和组合导航分别运行与不同的单片机中基于串口DMA进行高速数据交互
板载两套6轴惯性传感器、1个三轴磁力计、1个气压计并支持外部罗盘接入。组合导航CPU采用UCosII嵌入式操作系统基于卡尔曼滤波算法实现对GPS、光流、UWB和气压计数据的可靠
融合，从而实现室内外可靠的悬停和航线飞行，姿态和高度控制采用自抗扰（ADRC）控制算法实现对外部扰动的可靠控制同时具有响应快、信号跟踪性能好的特点，通过对自抗扰算法
改进实现了基于飞行器轴距、姿态、航向和高度三通道感度和快速调参。飞控在内部封装SDK二次开发接口和部分Demo，能快速实现一键起飞降落，视觉降落，目标跟踪和自主避障，
另外预留多个扩接口能作为地面机器人、无人车和无人船的硬件载体。飞控源码移植了Mavlink航向设置源码能实现基于Qground和MissonPlanner的任意航点、高度和速度的设置，
基于匿名地面站能实现对飞控内部任意融合结果、传感器参数、控制反馈期望和状态信息的实时显示和参数调节，基于板载NRF2.4通讯芯片能与地面手持遥控实现最远900米
的数据交互，实时显示飞行器经纬度、姿态，并对任意参数进行在线设定和修改，免去室外参数调节需要携带电脑和平板的不便。<br>

<div align=center><img width="240" height="240" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/remote.png"/></div>


**飞控特性**：<br>
	*UCosII操作系统<br>
	*自抗扰姿态控制<br>
	*卡尔曼组合导航<br>
	*SDK快速开发<br>
	*Mavlink航线规划和匿名地面站快速调参<br>
	*移动遥控端状态显示和参数在线修改<br>
	*GPS导航、视觉导航、自动降落、光流图像定位<br>

**飞控性能演示视频连接：**<br>
	[SDK开发演示](https://v.youku.com/v_show/id_XMzc4MzAyMDU4MA==.html?spm=a2hzp.8244740.0.0&f=49551028) <br>
	[室外GPS航线测绘和地面站航点设置](https://i.youku.com/i/UMTU1NTgzMzYw?spm=a2hzp.8253869.0.0) <br>
	[视觉固定目标降落](https://v.youku.com/v_show/id_XMzk2OTMyNzE4MA==.html?spm=a2hzp.8253869.0.0) <br>
	[视觉移动小车降落](https://v.youku.com/v_show/id_XMzc0MTI4NzQ2NA==.html?spm=a2hzp.8253869.0.0) <br>
	[室内二维码地标阵列定位](https://v.youku.com/v_show/id_XMjc2MDAxMDI1Mg==.html?spm=a2hzp.8253869.0.0) <br>
	[室外光流悬停](https://v.youku.com/v_show/id_XMjc1NzI1NDQ4NA==.html?spm=a2hzp.8253869.0.0) <br>
	[室内气压计定高](https://v.youku.com/v_show/id_XMjcyNjM2MzU5Mg==.html?spm=a2hzp.8253869.0.0) <br>

# 3 PCB硬件参数
  OLDX-FC硬件采用4层板设计，通过外部电源模块进行供电支持2S~4S电池供电，具有最大12路PWM输出4路AD信号输入，板载NRF2.4通讯芯片，预留6路串口1路CAN接口<br>
  
**硬件参数：**

项目|参数
-------------|-------------
处理器|STM32F405RGT6*2
处理器性能|32Bit ARM Cortex-M4 168MH
陀螺仪 加速度计|ICM20602 + LSM6DS33
磁力计|LIS3MDL
气压计|MS5611
预留接口|GPS*1 串口*4 CAN*1 图像*1
PWM 输出通道|8 通道PWM + 4 路AUX
供电|5V输入 IO输出5V
飞行器类型|四旋翼 六旋翼 八旋翼 共轴六旋翼
高度悬停精度|±0.02m（超声波） ±0.1m（气压计）
位置悬停精度|±0.2m（GPS） ±0.1m（光流）

# 4 软件说明
  OLDX-FC基于C语言和Keil5进行开发，飞行控制部分基于匿名早期裸奔程序架构，采用状态机调度保证不同线程的运行周期，
对其姿态控制部分进行修改，采用SO3下的旋转矩阵求取外环控制误差，姿态内环采用改进ADRC控制器保证对给的角速度的
稳定跟踪；高度控制部分替换原始互补滤波融合算法采用扩展卡尔曼滤波器融合气压计和加速度计，同样使用ADRC控制器控制
高速速度环。位置方面通过串口数据接收组合导航模块解算机体速度和位置，采用位置+速度+加速度三环控制飞行器位置；
通讯方面在保留匿名上位机调参功能外增加2.4G无线通讯，可脱离遥控器采用体感进行飞行器控制，另外移植Mavlink通讯协议实现
与Qground和MissonPlanner地面站的通信，实现室外飞行器航点设置和轨迹显示；增加SDK二次开发接口，封装了多种常用函数，如
速度给定，位置移动给定，航向飞行，图像目标对准，图像目标跟踪，地标引导降落的多个子API，通过简单的流程书写既可以实现
复杂的智能导航、图像导航功能，十分适合于Demo研发、电子竞赛、无人机竞赛和DIY开发中。<br>
组合导航模块基于UCOSII操作系统，基于UKF和KF算法完成GPS、UWB、光流与加速度传感器数据的融合，采用非线性AHRS算法实现
可靠的姿态解算和机体加速度解算，同时预留CAN总线接口方便后续外扩其他传感器数据。

<br><br>
**飞行控制模块**
项目|参数
-------------|-------------
姿态解算|互补滤波
高度融合|扩展卡尔曼(PX4)/抗差卡尔曼
姿态控制|SO(3)误差(PX4)+PD(角度)+ADRC(角速度)
高度控制|PD(高度)+ADRC(垂直速度)
位置控制|PD(高度)+PID(机体速度)+P(机体加速度)
通讯接口|2.4G无线通讯(匿名+OLDX手持遥控器) 串口数传(匿名+Mavlink)
控制方式|遥控器(PPM+SBUS) OLDX手持遥控器 SDK自主飞行
外部控制信号|4路舵机输出  支持使用飞控姿态控制两轴舵机云台稳相和无刷云台目标跟踪(RobotMaster)
地面站支持|匿名地面站(参数显示+波形显示+参数设置)  Qground/MissionPlanner(位姿轨迹显示+航点写入)

<br><br>
**组合导航模块**
项目|参数
-------------|-------------
姿态解算|非线性AHRS/梯度下降/扩展卡尔曼/互补滤波
位置融合|抗差卡尔曼/无迹卡尔曼(AutoQuad)
传感器接口|GPS(NEO-8M 乐迪迷你)+UWB(INF)+光流(Pixflow/OLDX-AMF)+超声波(串口/PWM)+激光测距仪(VL53L0X)

# 5 飞控使用教程
## 5.1 PCB接口说明

<div align=center><img width="540" height="460" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pcb.jpg"/></div>
<br><br>

接口|说明|支持模块
-------------|-------------|-------------
姿态解算|互补滤波|11




## 5.2 飞控宏定义和软件配置说明

## 5.3 控制参数调整和飞行器配置说明


## 5.4 首次飞行说明(四轴机型为例)
### 5.4.1 飞行器安装

# 6 进阶SDK开发说明

```
//代码块
int a=0;
a++;
```


# 7 捐赠与项目后续开发计划
如果您觉得该项目对您有帮助，也为了更好的项目推进和软硬件更新，如果愿意请通过微信捐赠该项目！
<div align=center><img width="240" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.png"/></div>





