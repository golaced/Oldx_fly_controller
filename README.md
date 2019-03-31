
# 1 OLDX-开源多旋翼开发平台项目  
<div align=center><img width="600" height="130" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/logo.JPG"/></div>
<div align=center><img width="440" height="280" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/fc.jpg"/></div>
____OLDX多旋翼开发平台（OLDX-FC）是由北京理工大学自动化学院所属《北理云逸科技》团队开发的一个目前国内最完整的免费开源飞控项目，随着国内开源飞控的逐步发展如匿名、
INF、无名和ACFly飞控的陆续推出，如光流、气压计和GPS等相关算法已经逐步完善，但是相比Pixhawk等国外开源飞控平台的发展和定位仍然有发展空间。OLDX-FC于14年开始对多旋翼飞行器进行研究期间也经历过开源和借鉴的过程，为希望进一步推动国内开源飞控协作开发和
相互学习、相互分享的趋势，团队将该OLDX-FC转化为开源项目，采用自由捐赠的形式继续发展。<br>
<div align=center><img width="440" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.png"/></div>
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
**-如果您愿意分享对该项目的优化和改进请联系golaced@163.com或加入我们的QQ群567423074，加速开源项目的进度-**<br>

	
# 2 基本功能介绍
____OLDX-FC是一个基于STM32F4系列单片机的多旋翼飞控平台，其采用双处理器、双IMU冗余的设计，飞行控制和组合导航分别运行与不同的单片机中基于串口DMA进行高速数据交互
板载两套6轴惯性传感器、1个三轴磁力计、1个气压计并支持外部罗盘接入。组合导航CPU采用UCosII嵌入式操作系统基于卡尔曼滤波算法实现对GPS、光流、UWB和气压计数据的可靠
融合，从而实现室内外可靠的悬停和航线飞行，姿态和高度控制采用自抗扰（ADRC）控制算法实现对外部扰动的可靠控制同时具有响应快、信号跟踪性能好的特点，通过对自抗扰算法
改进实现了基于飞行器轴距、姿态、航向和高度三通道感度和快速调参。飞控在内部封装SDK二次开发接口和部分Demo，能快速实现一键起飞降落，视觉降落，目标跟踪和自主避障，
另外预留多个扩接口能作为地面机器人、无人车和无人船的硬件载体。飞控源码移植了Mavlink航向设置源码能实现基于Qground和MissonPlanner的任意航点、高度和速度的设置，
基于匿名地面站能实现对飞控内部任意融合结果、传感器参数、控制反馈期望和状态信息的实时显示和参数调节，基于板载NRF2.4通讯芯片能与地面手持遥控实现最远900米
的数据交互，实时显示飞行器经纬度、姿态，并对任意参数进行在线设定和修改，免去室外参数调节需要携带电脑和平板的不便。<br>

<div align=center><img width="540" height="330" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/oldx.jpg"/></div>



**飞控特性**：<br>
	*UCosII操作系统(正点原子)<br>
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
____OLDX-FC硬件采用4层板设计，通过外部电源模块进行供电支持2S~4S电池供电，具有最大12路PWM输出4路AD信号输入，板载NRF2.4通讯芯片，预留6路串口1路CAN接口<br>
  
**硬件参数：**

项目|参数
-------------|-------------
处理器|STM32F405RGT6-2
处理器性能|32Bit ARM Cortex-M4 168MH
陀螺仪 加速度计|ICM20602 + LSM6DS33
磁力计|LIS3MDL
气压计|MS5611
预留接口|GPS-1 串口-4 CAN-1 图像-1
PWM 输出通道|8 通道PWM + 4 路AUX
供电|5V输入 IO输出5V
飞行器类型|四旋翼 六旋翼 八旋翼 共轴六旋翼
高度悬停精度|±0.02m（超声波） ±0.1m（气压计）
位置悬停精度|±0.2m（GPS） ±0.1m（光流）


<br><br>
**飞控外壳：**<br>
____提供飞控3D打印外壳STL文件，设计为气压计增加缓冲空间并设计了减震球底座。
<div align=center><img width="260" height="160" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/cap1.JPG"/></div>
<div align=center><img width="260" height="160" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/cap2.JPG"/></div>
<div align=center><img width="260" height="160" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/cap3.JPG"/></div>


# 4 软件说明
____OLDX-FC基于C语言和Keil5进行开发，飞行控制部分基于匿名早期裸奔程序架构，采用状态机调度保证不同线程的运行周期，
对其姿态控制部分进行修改，采用SO3下的旋转矩阵求取外环控制误差，姿态内环采用改进ADRC控制器保证对给的角速度的
稳定跟踪；高度控制部分替换原始互补滤波融合算法采用扩展卡尔曼滤波器融合气压计和加速度计，同样使用ADRC控制器控制
高速速度环。位置方面通过串口数据接收组合导航模块解算机体速度和位置，采用位置+速度+加速度三环控制飞行器位置；
通讯方面在保留匿名上位机调参功能外增加2.4G无线通讯，可脱离遥控器采用体感进行飞行器控制，另外移植Mavlink通讯协议实现
与Qground和MissonPlanner地面站的通信，实现室外飞行器航点设置和轨迹显示；增加SDK二次开发接口，封装了多种常用函数，如
速度给定，位置移动给定，航向飞行，图像目标对准，图像目标跟踪，地标引导降落的多个子API，通过简单的流程书写既可以实现
复杂的智能导航、图像导航功能，十分适合于Demo研发、电子竞赛、无人机竞赛和DIY开发中。<br>
____组合导航模块基于UCOSII操作系统，基于UKF和KF算法完成GPS、UWB、光流与加速度传感器数据的融合，采用非线性AHRS算法实现
可靠的姿态解算和机体加速度解算，同时预留CAN总线接口方便后续外扩其他传感器数据。

<br><br>
**飞行控制模块**

项目|参数
-------------|-------------
姿态解算|互补滤波(匿名)
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
姿态解算|非线性AHRS/梯度下降/扩展卡尔曼/互补滤波(匿名)
位置融合|抗差卡尔曼/无迹卡尔曼(AutoQuad)
传感器接口|GPS(NEO-8M 乐迪迷你)+UWB(INF)+光流(Pixflow/OLDX-AMF)+超声波(串口/PWM)+激光测距仪(VL53L0X)

# 5 飞控使用教程
## 5.1 PCB接口说明

<div align=center><img width="540" height="460" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pcb.jpg"/></div>
<br><br>

接口|说明|支持模块
-------------|-------------|-------------
飞控下载|飞控模块SWD下载口|download_fc模块 C->SCLK D->SWD
导航下载|导航模块SWD下载口|download_stlink模块
导航串口1|GPS和外部罗盘IIC接口|乐迪M8N Mini GPS C->SCL D->SDA
导航串口5|光流传感器接口|Pixflow  OLDX-AMF
导航串口3|超声波接口|US100 (串口模式下T->T R->R)  北醒激光测距模块
导航串口4|预留传感器接口|
导航CAN|预留CAN总线接口|
飞控串口3|图像处理接口|树莓派  Odroid-XU4 （图像处理器需自行供电）
飞控串口1|数传接口|匿名数传 3DR数传  CUAV WIFI数传
PWM1~8|电调接口|400Hz
AUX1~4|电调9~12/舵机控制接口 1俯仰 2横滚 3投递器开关|两轴舵机云台 两轴无刷云台
AD1~4|模拟电压采集接口|压力传感器AD(0~3.3V)
SBUS|接收机接口|天地飞接收机 Futaba接收机
舵机供电选择|R39外部供电 R38飞控供电 （任选一）|
供电|采用6P自锁双头端子线与供电模块连续|power模块 +->DC B->蜂鸣器信号 5->5V降压输入

<br><br>
**OLDX飞控最少所需配置**

模块组合|实现功能
-------------|-------------
飞控+Power模块|手动飞行/气压定高
飞控+Power模块+光流|手动飞行/气压定高/超声定高/光流悬停/SDK飞行
飞控+Power模块+GPS|手动飞行/气压定高/GPS航线飞行/SDK飞行
飞控+Power模块+GPS/光流+树莓派/Odroid|手动飞行/气压定高/GPS航线飞行/SDK飞行/图像导航/视觉降落

<br><br>
**接线示意图**
<div align=center><img width="540" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/line.jpg"/></div>



## 5.2 程序宏定义和飞控配置说明
### 5.2.1 遥控器通道

通道|功能|通道说明
-------------|-------------|-------------
CH1|横滚通道|
CH2|俯仰通道|
CH3|油门通道|
CH4|航向通道|
CH5|SDK模式使能|  通道值<1500(关闭自动SDK飞行) 通道值>1500(使能自动SDK飞行)
CH6|返航和自动起飞| 通道值<1500(打开AUX3口开关) 通道值>1500(关闭AUX3口开关)
CH7|位置模式|  通道值<1500(手动) 通道值=1500(速度悬停) 通道值>1500(位置悬停)
CH8|高度模式|  通道值<1500(超声波 气压计自动切换) 通道值=1500(气压计) 通道值>1500(手动)

**飞行器解锁上锁**：外八遥控操作<br>
**飞行中关闭遥控器**：自动返航/自动降落/电机急停<br>
**自动起飞和智能飞行**：CH5>1500 CH6>1500 CH7>1500 CH8<1500  状态下外八解锁  并把油门置于中位(自主飞行中任意遥感不在中位均会进入自主飞行模式，
回复中位后继续执行当前任务。需要取消飞行则保证CH5<1500) <br>
**自主任务状态机重置**：在飞行器执行自主任务后无论自动降落或者人工打断都需在着陆上锁后保证CH5<1500 CH7<1500<br>
**飞行中自动返航**：无论在自主飞行或人工遥控飞行中 如果CH6通道值从大于1500切换到小于1500则进入失控策略，过程中可以通过人为遥控打断，并重新进行触发<br>
**陀螺仪校准**：CH8<1500 时CH7从小于1500到大于1500 快速切换多次<br>
**磁力计校准**：CH8>1500 时CH7从小于1500到大于1500 快速切换多次，进入模式后BB响持续发声，蓝色1s间隔闪烁<br>

## 5.3 飞行器配置和控制参数调整说明
### 5.3.1 飞行器配置
（1）飞控模块include.h<br>

宏定义|说明
-------------|-------------
USE_OLDX_REMOTE|使用OLDX手持遥控器代替传统遥控器
AUTO_MISSION|使能SDK飞行 
MAXMOTORS|最大电机数量（4/6/8/12）
FLASH_USE_STM32|使用STM32内部EPRoom模拟Flash  使能则无法存储GPS航点
YAW_INVER|航向控制输出方向 用于电机转向与标准方向不对下的软件调整，不用重新安装电机
YAW_FC|不使用磁力计修正航向
MAX_CTRL_ANGLE|最大控制姿态角度
MAX_CTRL_YAW_SPEED_RC|最大控制航向角速度
ESO_AI|姿态控制内环ADRC b0参数（0表示不使用ADRC控制器）
ESO_YI|航向控制内环ADRC b0参数（0表示不使用ADRC控制器）
HOLD_THR_PWM|预设悬停油门
DEBUG_MODE|室内则封闭PWM输出，解锁后电机不转动可以作为室内Debug使用
TUNING_ONE_AXIX|使能则参数调节时仅针对一个轴
TUNING_X/ TUNING_Z|单轴调参目标
USE_KF|使用带估计加速度偏差的卡尔曼滤波器估计高度否则使用PX4提供的EKF高度估计算法
USE_CARGO|使用AUX3的舵机投递器

（2）飞控模块oldx_api.h<br>

宏定义|说明
-------------|-------------
LAND_SPD|自动降落速度
MAX_WAYP_Z|最大航点高度限制
WAY_POINT_DEAD1|航点达到判断死区
LAND_CHECK_G|着地检测重量加速度
LAND_CHECK_TIME|着地检测条件判断时间
YAW_LIMIT_RATE|旋转航向最大角速度限制 


（3）导航模块include.h<br>

宏定义|说明
-------------|-------------
USE_UKF_FROM_AUTOQUAD|使用Autoquad提供的UKF融合算法(存在Bug)
UKF_IN_ONE_THREAD|UKF融合时不使用UcosII系统
USE_US100/ USE_KS103/ USE_LIDAR|定高传感器数据选择
SONAR_SAMPLE1/ SONAR_SAMPLE2/ SONAR_SAMPLE3|高度传感器数据采样频率
USE_IMU_BACK_IO_AS_SONAR|使用串口4采集高度传感器数据
SONAR_USE_FLOW|直接使用Pixflow传感器自带超声波高度数据

<br><br>
**飞行器型号设置**：<br>
（1）采用如下宏定义定义你的飞行器<br>

```
#if defined(M_DRONE)
#define M_DRONE_ID     //单片机id_chip值
#define PX4_SDK 0  
//#define PX4_LINK
#define USE_MINI_FC_FLOW_BOARD 0 
#define USE_VER_8_PWM 1  
#define USE_VER_8 0  
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
//#define USE_CARGO     //可修改
#define AUTO_MISSION    //可修改

//#define USE_KF	    //可修改 
#define USE_UWB
#define MAXMOTORS 		(4)		//可修改
#define FLASH_USE_STM32 0       //可修改
#define YAW_INVER 	  0		    //可修改
#define YAW_FC 0  				//可修改
#define MAX_CTRL_ANGLE			30.0f	//可修改
#define MAX_CTRL_YAW_SPEED_RC   200	    //可修改

#define ESO_AI 22  //可修改
#define ESO_YI 22  //可修改
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(500,0,500)  //可修改
#endif
```
<br>
并在24~42行"唯一"定义该机型：
<br>

```
#define M_DRONE
//#define M_DRONE250X4
//#define M_DRONE_330X6
//#define M_DRONE_330 
//#define M_DRONE_PX4
//#define M_CAR
```

<br>
并在init.c 43行定义飞控对应2.4G通讯通道CHE，修改188行 mode_oldx.rc_loss_return_home选择失控模式(0上锁 1返航 2降落 3无视)
<br>

```
switch(id_chip)
{
case M_DRONE_ID: CHE=25;break;
case IMAV2: CHE=22;break;
case IMAV3: CHE=33;break;
default: CHE=11;break;
}
```

<br>
并在pos_ctrl.c 854行定义使用的SDK：
<br>

```
switch(mission_sel_lock)
{
case 0:
switch(id_chip)
{
case M_DRONE_ID: mission_flag=mission_test_gps(T); break;//你的SDK
case IMAV2: mission_flag=mission_search(T); break;
case IMAV3: mission_flag=mission_test_gps(T); break;
default:mission_flag=mission_test_gps(T); break;
}	
break;
```

<br>
并在scheduer.c 152行定义UART_UP_LOAD_SEL选择上位机波形显示队列
<br>

### 5.3.2 参数调节
____飞控已经集成基础参数调节功能在完成飞行器机型设置和固件更新后，进行传感器校准即刻起飞或者进行参数调节。<br>
(1)远程调参与波形显示<br>
____飞控默认采用2.4G无线与OLDX手持遥控端进行通讯，参数调节可以直接通过遥控器实现，另外手持遥控器同时具有USB虚拟串口，
遥控器工作后连接PC机则可以使用匿名地面站进行参数调节(115200默认波特率)和参数波形显示。(注：对手持控制器来说需要在PID参数界面
等等数据全部接受完成后在进行调参，否则会出现参数误写入引起的炸鸡；对地面站同样也是建议在数据通信正常后最少读取5次PID参数避免误写入问题)
<div align=center><img width="640" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/tunning.jpg"/></div>

(2)姿态参数调节<br>
____飞控安装好后首先需要进行姿态参数的调节推荐采用万向轴或烤四轴的方式固定飞行器进行参数调节，通过选择调参模式确认是单轴还是全向调参：

```
TUNING_ONE_AXIX 0
TUNING_X 0
TUNING_Z 0
```

默认都为0则飞行器全向参数都可调节，则以该情况为例设置 UART_UP_LOAD_SEL=11 使能上位机波形显示为姿态参数调节序列，通过CH6(>1500外环)能选择显示内环还是
外环的期望和给的参数，则波形中Ax Ay Az为期望的横滚 俯仰 航向值，Gx Gy Gz为对应反馈值，Hx Hy Hz为对应误差。参数调节时如果外环PID参数有值则
遥杆给定期望角度，如无值则给定期望角速度。<br>

A.如飞行器机架小于550轴距可直接调整感度来快速完成参数调节，使用手动模式起飞保证飞行器离地观测姿态控制是否响应迅速，
是否存在超调和抖动，感度对应PID参数为：<br>

参数|说明|默认值
-------------|-------------|-------------
PID17-P|横滚、俯仰感度(增大则等于增大飞机调节增益)|1000
PID17-I|航向感度|1000
PID17-D|高度感度|1000

<br><br>

飞行器机型|主要动力配置|飞行器感度
-------------|-------------|-------------
330X4四轴|3s 4000mah  1500kv*4 好赢乐天15A 8寸桨|俯仰:1000    航向:1000   高度:1000   b0:22
450X4四轴|4s 4000mah  920kv*4 好赢乐天20A 9寸桨|俯仰:820    航向:1000   高度:1100   b0:22
6轴泡沫机|3s 10000mah  920kv*6  Emax 40A  8寸桨|俯仰:300    航向:200   高度:100  b0:3.5
共轴反桨6轴涵道|6s 10000mah 450kv*12 Emax40A 9寸桨|俯仰:250   航向:280   高度:68   b0:2.2
6轴飞行器+小车底盘|6s 16000mah *2 180kv *6 Emax40A 14寸桨|俯仰:325 航向:168 高度:100 b0:0 II:400 ID:4500 OP:368 P_GAIN:100

B.如飞行器轴距较大，电机KV值较小则不推荐采用起飞调参的方式，在调试架上固定好飞行器后采用手动模式解锁调参并内外环波形曲线。
调参时首先将ADRC控制器b0置0不使用其控制输出，将外环PID清0仅调整角速度环，解锁后观察飞行器是否能较好地跟踪遥控给定速度，
在能保证基本跟踪上后将姿态ADRC b0在22-300进行调整(默认220)，如果飞机出现晃动则减小，出现顿挫则增大，在基本实现跟踪期望速度后(可存在一定
滞后，特别是大飞机)通过拉拽飞行器轴造成外部扰动和控制器饱和来确定是否需要增大D，如果出现按压时的超调可增大D来保证回弹的柔和。<br>
在完成内环参数调节后，设置外环P参数(默500-700)，在保证出现一定超调下的P时增加D达到最终回中无快速无超调(550以下小飞机可能不需要D)，之后
同样采用人为拉拽或按压测试控制器饱和回中是否出现超调和震荡判断参数是否合适。<br>
对航向来说一般仅需要调整感度既可以满足一般应用，如有更高性能要求可采用类似如上的方式进行参数调节，另外航向ADRC b0参数推荐仅在小轴距飞行器
上使用，则姿态调节对应PID参数表如下：<br>


参数|说明|默认值
-------------|-------------|-------------
PID1-P|横滚、俯仰 内环P|700
PID1-I|横滚、俯仰 内环I|200
PID1-D|横滚、俯仰 内环D|2200
PID2-P|姿态ADRC  b0|220
PID2-I|航向ADRC  b0|220
PID2-D|ADRC 控制死区|268
PID3-P|航向 内环P|1200
PID3-I|航向 内环I|100
PID3-D|航向 内环D|1200
PID4-P|横滚、俯仰 外环P|700
PID4-I|横滚、俯仰 外环I|0
PID4-D|横滚、俯仰 外环D|0
PID6-P|航向 外环P|800
PID6-I|航向 外环I|50
PID6-D|航向 外环D|300

在完成参数调节后可以采用两种方式将其保存在Flash中，对于感度可以采用遥控触发陀螺仪的方式写入(水平静置)，但还是推荐采用在代码里修改的方式。
其中对于ADRC b0参数在机型宏定义中修改，飞行器感度在Debug watch中修改后选择校准陀螺仪(见6.4.2小节)保存，姿态PID则在parameter.c 40行中进行修改:

```
float pid_att_out[3]={0.7,0.0,0.0};
float pid_att_in[3]={0.7,0.2,2.2};

float pid_att_out_yaw[3]={0.8,0.05,0.3};
float pid_att_in_yaw[3]={1.2,0.1,1.2};
```

(3)高度参数调节<br>
____高度参数调节方式类似姿态调节，如对性能要求不高则可采用默认参数通过调整感度的方式实现快速起飞。另外也可以采用上位机
对比期望波形的方式调节，将UART_UP_LOAD_SEL=2显示高度控制波形序列，之后同样采用将ADRC b0置0先内环后外环的方式，调节时
注意保证姿态参数已经可靠，出现震荡时快速切换到手动模式降落。

```
scheduler.c 282行
case 2://高度控制，速度z控制和速度xy
data_per_uart1(
ALT_POS_BMP_UKF_OLDX*1000,ultra_ctrl.exp,baro.h_flt*100,
ALT_VEL_BMP_UKF_OLDX*1000, wz_speed_pid_v.exp,wz_speed_pid_v.now,
VEL_UKF_Y*100,VEL_UKF_X*100,0,
(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
```											


参数|说明|默认值
-------------|-------------|-------------
PID7-P|高度 内环P|500
PID7-I|高度 内环I|300
PID7-D|高度 内环D|1680
PID8-P|高度 外环P|1000
PID8-I|高度 外环I|100
PID8-D|高度 外环D|0
PID12-P|高度 ADRC b0|15
PID12-I||
PID12-D|高度 ADRC 死区|10

保存参数类似姿态控制的方式，参数在程序如下部分：

```
height_ctrl.c 41行

ultra_pid.kp = 1.0;
ultra_pid.ki = 0.1;
ultra_pid.kd = 0.0;

wz_speed_pid.kp = 0.5;
wz_speed_pid.ki = 0.3;
wz_speed_pid.kd = 1.68;
wz_speed_pid.fp=0.2;

//adrc
eso_pos_spd[Zr].b0=15;
eso_pos_spd[Zr].eso_dead=0.01*1000;	
eso_pos_spd[Zr].eso_for_z=1;

```			

(4)位置参数调节<br>
____位置参数调节方式可采用上述调参方式其波形序列为UART_UP_LOAD_SEL=3(速度)，UART_UP_LOAD_SEL=5(位置)。
调节时注意保证姿态参数已经可靠，出现震荡时快速切换到手动模式降落。如定高参数以稳定可采用气压计定高起飞，
在达到齐胸高度后先切换速度模式，给定姿态观测回中后是否有可靠制动是否存在刹车时晃动，如角度太大则减小速度内环P即可(其他参数可不调节),
在速度环制动平缓柔和且悬停无明显晃动后，切入位置模式观测是否还存在漂移，通过给的速度制动观察飞行器控制效果，对小飞机仍然
可采用人为拉拽的方式验证。

参数|说明|默认值
-------------|-------------|-------------
PID9-P|位置 内环P|330 (对大轴距飞行器可降低到180)
PID9-I|位置 内环I|0
PID9-D|位置 内环D|0
PID10-P|位置 外环P|360
PID10-I|位置 外环I|50
PID10-D|位置 外环D|0


保存参数类似姿态控制的方式，参数在程序如下部分：

```
pos_ctrl.c 321行

nav_pos_pid.kp=0.36;
nav_pos_pid.ki=0.0500;
nav_pos_pid.kd=0.0;
nav_pos_pid.dead=0.01;

nav_spd_pid.f_kp=0.2;
nav_spd_pid.kp=0.33;
nav_spd_pid.ki=0;
nav_spd_pid.kd=0;
nav_spd_pid.max_exp=0;

nav_spd_pid.flt_nav=1;//修改为0.356左右可平滑制动手感  目前已经在程序中增加S函数保证制动平滑
```			

## 5.4 手持遥控器(OLDX-Remote)介绍
____为提高调参速度减少户外飞行所需PC机和上位机设备,OLDX飞控设计了一个手持端遥控器，其除了能使用体感模式遥控飞行器外，还能代替PC地面站进行实时参数调节，
另外其实时回传SDK主状态机和子状态机运行状态，期望高度速度信息和航线航点信息，能实时了解当前SDK运行状况和下一时刻飞行器的目标任务，解决了飞行器自主
飞行中对其内部状态难以获取的问题。
<div align=center><img width="640" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/remote.jpg"/></div>
### 5.4.1 界面介绍
____OLDX-Remote开机后会显示当前遥控器通道请保证其与飞控中一致（之前在线修改），遥控器目前具有三类界面，（1）主界面：显示飞行器高度，姿态，电压和飞行时间等常用数据（2）PID参数界面与PC端地面站对应显示飞行器内部参数（3）
SDK界面：显示航线和自主任务命令，状态机状态。则各界面下英文缩写如下：

主界面|说明
-------------|-------------
P|OLDX-Remote 俯仰遥杆值(体感)
R|OLDX-Remote 横滚遥杆值(体感)
T|OLDX-Remote 油门杆值(遥杆)
Y|OLDX-Remote 航向杆值(遥杆)
SR|飞控与OLDX-Remote间的通信强度
BR|OLDX-Remote电量
BF|飞行器电量(飞行中小于25%则需尽快降落)
M|飞行器模式 Mual->手动  ALT B->气压计 ALT S->超声波 Pos->位置参数调节
Bad Pos|定位模式状态  Bad Pos->无定位 GPS->卫星数量(推荐8颗以上起飞) Opt->光流质量(推荐120以上地面起飞)
lock|飞行器锁定情况
矩形界面中P|飞行器俯仰角
矩形界面中R|飞行器横滚角
矩形界面中Y|飞行器航向角
矩形界面中 0:0|解锁后飞行时间
矩形界面中X,Y,Z|飞行器局部X轴位置(米)
矩形界面中 三个正方形点| 从上到下 *GPS连接  *图像设备连接 *光流模块连接
中心圆圈和实心点|单成像中c2c结构体pix_x,y有数会显示原点，可使用其来判断图像目标十分识别和在图像中的像素位置

<br><br>

PID参数界面|说明
-------------|-------------
PIDX-X|上位机中对应参数
左右拨动遥感到底2s左右可切换界面|
左右上下选择PID参数|
点击遥控出现*则选中该参数|选中下左右增加100参数 上下增加10参数
选择下再次点击退出选中—|遥控器每5s读取飞控数据 选中下不读取 
选中中长按遥感遥控发出BB声则参数写入|对CHE遥控通道来说其写入 需到主界面进行相同操作 同时会校准遥控IMU

<br><br>

SDK界面|说明
-------------|-------------
main|Idle(状态机默认状态 该状态自动起飞后才会运行SDK)  Mission(SDK模式)  Safe(状态机保护进入普通悬停模式  需降落上锁后复位所用遥控开关方可清除)
RC右方%|飞行器电量
Subs|子状态机状态
Way|剩余航点数量
左方竖条|高度期望和反馈  位置模式(期望为横线当前为矩形中心) 速度模式(显示上升下降期望速度)
矩形框|位置期望和反馈   中心为当前飞行器位置，原点为期望位置(坐标系为x-y  东-北)，坐标系尺度自动缩放，小横线表示飞行器机体方向
Dis|下一个航点距离(米)



### 5.4.2 遥控器模式介绍
OLDX-Remote板载MPU6050 IMU传感器可实现体感操作，在include.h中修改如下：

```
#define USE_RECIVER_MINE 1
```

体感操作遥控器遥感上下对应油门，左右对应航向，前后倾斜对应俯仰，侧斜对应横滚。飞行器解锁为遥感右下2秒。通过在主界面长按遥感按键
可以实际对遥控器中位的零偏校准。


# 6 首次飞行说明(四轴机型为例)
## 6.1 飞控安装和程序下载
____首先将OLDX飞控沿机头方向安装在机体中心，推荐使用减震处理，(不在中心可能由于旋转效应造成而外加速度影响制动或悬停)，下图给出了典型飞行器的电调PWM
信号线顺序和螺旋桨转向，如电机转向与图中相反可以重新安装或者定义YAW_INVER为1：

<div align=center><img width="640" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/drone.jpg"/></div>

安装好电调和飞控后，将Power模块使用6P双头自锁线与飞控供电端连接，同时将Power端DC输入与飞行器整体供电输入焊接在一起(2S~4S),将螺旋桨解除，采用如下图的方式
使用download和ST-link下载器连接飞控供电：
<div align=center><img width="540" height="340" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/SWD.jpg"/></div>

____在确认ST-Link驱动安装无误后首先向FC飞控单片机下载程序，下载完成后可以采用Debug模式运行或者重新上电，如系统正常则蜂鸣器会响并且LED闪烁，之后采用同样方法
向IMU导航单片机下载程序，运行后查看Watch中lis3mdl结构体(如没有请手动右键添加，同时保证菜单View下拉中Periodic Update选择)中Acc_I16、Gyro_16、Mag_Adc是否是数
如有数(传感器正常)则将电路板水平静置将Acc_CALIBRATE和Gyro_CALIBRATE分别置1，查看Acc_Offset和Gyro_Offset保存的当前传感器零偏，复位芯片后查看读书是否基本一致
验证板载Flash芯片正常。<br>
____完成IMU下载和电路验证后重新Debug飞控单片机，此时开机蜂鸣器应当会发出开机音乐,同样mpu6050_fc结构体中传感器参数是否刷新，校准传感器复位芯片检测Flash中
是否读出上次标定参数，如全部正常则硬件部分基本无误可以尝试飞行。<br>

注：如将如下部分进行修改则开机可以模拟Pixhawk蜂鸣器声音

```		
//beep.c 145行
case START_BEEP:
music_sel=	START_BEEP;
Play_Music(start_music_windows,0,sizeof(start_music_windows)/2);	
break;
		
//修改为
case START_BEEP:
music_sel=	START_BEEP;
Play_Music(start_music_px4,0,sizeof(start_music_px4)/2);	
break;
			
```		

**另外飞控安装时最好使用黑色泡沫覆盖气压计部分并3D打印外壳，防止飞行中气压计受气流干扰！！**

## 6.2 飞控配置和起飞前传感器校准
____在完成飞控安装和电路验证后，安装之前所述方式定义自己的飞行器和SDK，调整参数或采用默认参数飞行，首次飞行器需再次进行传感器校准。<br>
（1）首先将飞行器水平放置于地面，采用前文所述遥控开关校准加速度计和陀螺仪的方式标定安装零偏，完成后查看上位机或手持遥控器中姿态角是否慢慢归0；<br>
（2）之后同样采用遥控开关校准方式触发磁力计校准，进入校准过程中蜂鸣器会1秒1次响同时LED为蓝灯闪烁，此时拿起飞行器进行旋转确保飞行器每个方向旋转一次后
继续水平旋转观差LED等是否退出蓝色闪烁模式，则标定完成。同样在远程端查看航向角是否符合当地实际朝向(航向偏差可能造成飞行器悬停打圈晃动)。<br>
完成校准后通过外八解锁查看电机转动情况确认符合安装要求，推动油门缺电机转速变化，下图以天地飞8通道遥控器为例给出一个常用的遥控器开关映射图。

<div align=center><img width="540" height="400" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/CH.jpg"/></div>

**注：如采用图中遥控配置：<br>**
(1)则高度档在上快速拨动POS档会触发磁力计校准，高度档在下快速拨动POS会触发加速度和陀螺仪校准；<br>
(2)SDK在上，失控在右，高度在中，位置在上(确认主状态机复位)，则解锁后油门到中位触发自动起飞和SDK飞行；<br>
(3)SDK飞行降落后，仅将失控拧到左，位置在下，SDK在下即可完成对状态机的复位；<br>
(4)飞行中失控从右旋转到左则进入失控保护策略；<br>

## 6.3 LED状态显示
____OLDX飞控板载三色LED能实现对飞控系统状态的基本情况查看，具体LED闪烁情况如下表所示：

LED闪烁|说明
-------------|-------------
红灯快闪|未连接遥控器
白色呼吸灯|飞控状态正常
蓝灯快闪|磁力计校准

## 6.4 手动飞行
____在完成飞控安装，电调顺序确认，电机转向缺人和飞行器参数配置，传感器校准后可以通过手动飞行模式缺参数是否合适，其具体步骤如下：<br>
(1)上电等待遥控连接白色呼吸灯<br>
(2)查看手持遥控器确认姿态角正确<br>
(3)确保高度模式和位置模式为手动状态<br>
(4)解锁飞行器，轻推油门确认电机转向正确<br>
(5)推到油门寻找飞行器起飞油门<br>
(6)在确保飞行器离地0.5米拨动遥杆，在上位机观察反馈数据或者目视观察控制参数是否合适<br>


## 6.5 定高飞行
____在完成手动飞行后可进行定高测试：<br>
(1)确保飞行器正常工作后切换气压定高模式<br>
(2)解锁飞行器后推动油门到中位以上，飞行器起飞切遥杆对应上下速度<br>
(3)在确保飞行器离地0.5米拨动遥杆，在上位机观察反馈数据或者目视观察控制参数是否合适<br>


## 6.6 位置悬停
在完成定高飞行后可进行位置悬停测试：<br>
(1)确保飞行器正常工作后切换气压定高模式<br>
(2)确保GPS定位星数满足6颗以上或者光流传感器连接且Qual大于150并且安装朝向正确<br>
(3)解锁后定高起飞<br>
(4)在确保飞行器离地0.5米后切换到速度模式，查看飞行器是否具有制动能力，切换到位置模式查看是否能满足0.2m内的悬停精度

## 6.7 航线飞行
____OLDX飞控移植了Mavlink通讯协议，目前支持与Qground地面站通信比进行航点设置，由于通讯协议还在完善因此目前只支持如下功能：<br>
(1)飞行器基础状态查看：姿态角、高度、GPS位置、GPS卫星数量、飞行器电量<br>
(2)航点写入<br>
<br>
正在更新的功能为：<br>
**(1)航点读出**<br>
**(2)自动起飞和RTL返航**<br>
(3)传感器校准<br>
(4)参数调节<br>

下面主要介绍航点参数写入操作：<br>
<div align=center><img width="640" height="400" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/dmz.JPG"/></div>
<br>
各航点内相关参数说明如下：

参数|说明
-------------|-------------
Altitude|航点高度
Hold|航点停留时间
Heading|航点朝向
Acceptance|飞行速度 **(注：由于协议问题无法使用Flight Speed设置航点速度)**

____如要统一修改航线高度则先将第一个航点高度设置好后，后续添加航点高度将与其一致，写入航点后上位机会显示写入过程但由于通讯等问题可能
造成出现协议错误提示，此时取消后反复写入即可，如果航点写入正常则飞控会发出BB提示音，并可以在手持遥控器中SDK界面查看写入航点数量是
否一致。

## 6.8 SDK自主飞行
____SDK自主飞行时OLDX飞控相比目前市面上所有开源飞控最独特和最重要的功能，其旨在提供一个易于快速实现复杂控制逻辑和图像导航的编程方式，
后续将使用micropython作为SDK编程核心基于图形界面实现直观简单的SDK编程，其具体开发请参照第7小节内容，使能SDK飞行的具体过程如下：<br>
<br>
**(1)在飞行器宏定义中使用#define AUTO_MISSION，在Pos_ctrl.c文件中定义选用的SDK **<br>
(2)确认连接GPS或光流传感器<br>
**(3)在起飞前确认主状态机为IDLE**<br>
(4)遥控器开关使能SDK，打开位置模式和定高模式<br>
(5)解锁飞行器后推动油门到中位，确保其他遥感在中位则飞行器自动起飞并执行SDK飞行<br>
**(6)在飞行过程中可在手持遥控SDK界面实时观察自动任务执行情况，自主飞行中可通过遥控操作打断飞行或者使用失控反航**


# 7 进阶SDK开发说明
## 7.1 主状态机书写
____SDK开发只需要完成主状态机框架书写和SDK调用即可，在oldx_mission.c中定义你的SDK程序并在pos_ctrl.c中进行调用，具体的SDK框架如下：

```
u8 mission_test(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	static float mission_time;

	switch(mission_state){
	case 0:
			   flag=take_off_task1(2,1.5,dt);
		   break;
	case 1:
			   flag=set_drone_z_task(5.5,dt);
	       break;
	case 2:mission_finish=1;
	       break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	mission_time+=dt;
	
	if(mission_finish)
	  return 1;		
  else
      return 0;		
}	

```

上段代码给出了一个简单的自动起飞后悬停在5.5米高度的SDK程序。SDK以一个状态机进行驱动，每个状态中通过调用API使得flag为1表示该子
任务完成实现对状态标志为mission_state的向前推动，同一个状态下可以进行多个任务但需要保证flag由谁进行触发，同时也可不使用API完成的判断
自己设计状态跳转条件，状态跳转也可以不采用顺序进行对mission_state直接赋值实现特殊状态处理，该文件下给出了多个Demo实例，可参照其进行修改
实现SDK开发。

## 7.2 SDK介绍
这里给出目前版本中支持的API函数输入和输出详细说明：<br>

set_pos_task|设置飞行器全局目标位置
-------------|-------------
x,y,z|全局坐标系位置（米），如某个参数为0则保持当前该轴位置
完成条件|全局位置误差小于WAY_POINT_DEAD

<br><br>

way_point_task|设置飞行器GPS目标位置
-------------|-------------
lat,lon,height|GPS期望位置，如某个参数为0则保持当前该轴位置
loter_time|该航点悬停时间
mode|mode==MODE_FAST_WAY 则为快速飞行模式(放大对航点控制误差的判断条件)
完成条件|目标误差小于WAY_POINT_DEAD1并保证持续MISSION_CHECK_TIME时间

<br><br>

way_point_mission|航线飞行
-------------|-------------
spd_limit|总体航向速度限制(每个航点速度可在上位机中accpentance进行设置如其为0则使用该参数限制飞行速度)
head_to_waypoint|为1则机头朝向每个航点
mode|mode==MODE_FAST_WAY 则为快速飞行模式(放大对航点控制误差的判断条件)
完成条件|经过所有航点


<br><br>

return_home|返航
-------------|-------------
set_z|返航高度
spd_limit|速度限制
en_land|为1则返航后直接执行自动降落
head_to_home|机体朝向返航点
完成条件|完成返航任务

<br><br>

spd_move_task|机体下给定速度
-------------|-------------
x,y,z|给定速度，如为0则该轴保持当前位置
time|给定速度持续时间
完成条件|达到设定时间

<br><br>

pos_move_global_task|在全局坐标系下向前后左右移动距离
-------------|-------------
x,y,z|移动距离
完成条件|达到设定位置

<br><br>

pos_move_body_task|在机体坐标系下向前后左右移动距离
-------------|-------------
x,y,z|移动距离
完成条件|达到设定位置

<br><br>

pan_set_task|控制云台俯仰角度
-------------|-------------
pitch|设定角度
完成条件|无

<br><br>

pan_search_task|云台上下搜索API
-------------|-------------
min|最小角度
max|最大角度
da|角度增量
en_yaw|为1则搜索中航向也会左右反复搜索
en_forward|为1则每次完成一次反复搜索后向前以0.3m/s的速度前进
完成条件|无

<br><br>

down_ward_search_task|圆环轨迹搜索
-------------|-------------
r|圆轨迹半径
z|飞行高度
d_angle|圆形轨迹角度增量  
d_r|半径增量
完成条件|无


<br><br>

down_ward_search_task_tangle|蛇形航点轨迹规划
-------------|-------------
w|规划轨迹宽度航点数量
h|规划轨迹长度航点数量
wn|宽度间隔距离  
hn|长度间隔距离
spd_limit|飞行速度限制
z|飞行高度
完成条件|经过所有航点


<br><br>

target_track_downward_task|下置相机目标跟踪
-------------|-------------
target|目标图像信息结构体
spdz|下降速度
end_z|下降停止高度  
mode|模式 MODE_SPD：使用像素偏差  MODE_POS：使用估计的全局位置
完成条件|达到下降高度

<br><br>

target_track_pan_task|前置云台目标跟踪
-------------|-------------
target|目标图像信息结构体
close_param|速度模式下为云台最小角度  位置模式下为离目标距离
en_pitch|使能云台俯仰轴对准  
en_yaw|使能飞行器航向对准
close_mode|模式 MODE_SPD：使用像素偏差  MODE_POS：使用估计的全局位置
完成条件|云台达到目标角度或飞行器离目标距离达到设定值

<br><br>

avoid_task|自主避障
-------------|-------------
target|目标图像识别结果
set_x|期望前进X轴速度
set_y|期望前进Y轴速度
set_z|NC
dead|速度死区
max_spd|最大避障速度
完成条件|

<br><br>

avoid_way_point_task|航点自主避障
-------------|-------------
target|目标图像识别结果
tar_lat|目标经度
tar_lon|目标维度
spd|前进速度
dead|速度死区
max_spd|最大避障速度
mode|模式NC
完成条件|达到目标位置

<br><br>

set_drone_yaw_task|设置飞行器航向
-------------|-------------
yaw|目标航向角
rate_limit|旋转速度限制
完成条件|达到目标角度


<br><br>

set_drone_z_task|设置飞行器高度
-------------|-------------
z|目标高度
完成条件|达到目标高度

<br><br>

aux_open|辅助通开关设置
-------------|-------------
sel|辅助通道
open|开关
完成条件|


<br><br>

land_task1|降落
-------------|-------------
spd|下降速度
完成条件|达到触地检测标准

<br><br>

delay_task|空闲等待
-------------|-------------
delay|等待时间(秒)
完成条件|达到等待时间

<br><br>

draw_heart|光滑爱心图案
-------------|-------------
time|飞行总时间(秒)
size|爱心尺寸(米)
完成条件|达到等待时间

## 7.3 常用SDK解析
### 7.1 自动起飞->航线飞行->自动返航
____该SDK为最常用的任务架构，通过在其中添加打断航线飞行的状态判断能快速地实现飞行中对目标识别
触发的跟踪，或者飞行中看到降落标志触发的自动降落。

```
u8 takeoff_mission_return(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	static float mission_time;

	switch(mission_state){
	case 0:
			   flag=take_off_task1(5,1.5,dt);//自动起飞以1.5m/s的速度到达5m高度
		   break;
	case 1:
			   flag=delay(2,dt);//在该高度等待2s
	       break;
    case 2:
			   flag=way_point_mission(2,NEN_HEAD_WAY,dt);//以2m/s的速度进行航向飞行期间航向角为航点设定角度
		   break;
	case 3:
			   flag=return_home(15,2,EN_LAND,NEN_HEAD_WAY,dt);//以2m/s的速度从15m的高度反航，进行航向飞行期间航向角为航点设定角度并直接降落
	break;
	case 4:mission_finish=1;
	       break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	mission_time+=dt;
	
	if(mission_finish)
	  return 1;		
  else
      return 0;		
}	

```
### 7.2 自动起飞->区域搜索->自动返航
____该SDK是IMAV比赛中一个项目，项目要求如下：起飞前给定参赛选手5分钟前一个伤员的经纬度，无人机需要自主飞行到
该区域上方基于图像信息判断伤员位置，并将舵机挂钩上的医疗箱投递到伤员5m的半径内，之后自动返航并基于图像精确
降落在起飞点1m内。

```
float lat_human,lon_human;//寻找找伤员的经纬度
u8 mission_search1(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	static float mission_time;
	switch(mission_state){
	case 0:flag=take_off_task1(2,1.5,dt);//自动起飞 
		    break;
	case 1:pan_set_task(90,dt);//云台向下
		   flag=set_drone_z_task(6,dt);//达到起飞高度6m
		     mission_time=0;
		     break;	
  case 2:  mode_oldx.sdk_flag=1;//切换图像识别为伤员      
		   flag=way_point_mission(1.6,NEN_HEAD_WAY,dt);//开始航线飞行
	       if(c2c.connect&&c2c.x<160+90&&c2c.x>160-90&&c2c.y<120+90&&c2c.y>120-90)//识别到伤员
				   cnt_task[5]++;
				 
		   if(cnt_task[5]>20&&cnt_task[6]==0)//切换到目标对准
		  {mission_state=102;cnt_task[0]=cnt_task[1]=cnt_task[2]=cnt_task[5]=flag=0;}
		     break;
	case 3://在给定经纬度未找到目标则规划蛇形航线进行搜寻
				 mode_oldx.sdk_flag=1;
				 flag=down_ward_search_task_tangle(10,10,3,3,0.6,7.5,dt);
				 if(c2c.connect&&c2c.x<160+90&&c2c.x>160-90&&c2c.y<120+90&&c2c.y>120-90)//找到伤员
					 cnt_task[5]++;
				 
				 if(cnt_task[5]>6&&cnt_task[6]==0)//找到伤员切换对准状态
					{mission_state=102;cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}
					
				 if(flag&&cnt_task[6]==0)//搜寻完毕后重复直到超时
				  {mission_state=2;cnt_mission[MISSION_CNT]=cnt_task[5]=cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}
					
				 if(cnt_task[6]==1)//完成投递则返航
					 flag=1;
	       break;
  //------------------------------------------------------
	case 4:flag=return_home(12,2,NEN_LAND,NEN_HEAD_WAY,dt);//返航
				 if(flag)
					cnt_task[0]=cnt_task[1]=cnt_task[2]=0;
	       break;
	case 5:flag=set_drone_z_task(6,dt);//达到识别降落标志最佳高度
				 break;
	case 6:mode_oldx.sdk_flag=0;//切换图像识别为降落标志
				 flag=target_track_downward_task(&c2c,0.4,0.66,MODE_SPD,dt);//使用下置相机对准降落标志
				 if(c2c.check)
				  cnt_task[0]=0;
				 else
				  cnt_task[0]++;
				 
				 if(cnt_task[0]>10/dt)//降落过程中一直未找到目标则切换搜索模式
				 {mission_state=11;cnt_task[0]=cnt_task[1]=cnt_task[2]=0;
				 cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=flag=0;}
				
				 if(cnt_task[1]++>15/dt)
				 {flag=1;cnt_task[0]=cnt_task[1]=0;}
				 break;
	case 7:mode_oldx.sdk_flag=0;
				 flag=target_track_downward_task(&c2c,0.25,0.66,MODE_SPD,dt);
	       break;
	case 8:flag=land_task1(LAND_SPD,dt);//垂直降落
	       break;
	case 9:mission_finish=1;
	       break;
	//-----------------------AUX MISSION----------------------------
	case 11://搜索降落标志
	      mode_oldx.sdk_flag=0;//
	      flag=down_ward_search_task_tangle(8,8,4,4,0.4,5,dt);//tangle
			  if(c2c.check)
					 cnt_task[0]++;
				
				if(cnt_task[0]>5)
					{cnt_task[0]=0;mission_state=6;flag=0;}
					
				if(cnt_task[2]++>60/dt||flag)
				{mission_state=12;cnt_task[2]=0;flag=0;}
	      break;
	case 12://搜索一边仍未找到降落标志则直接返航使用起飞点经纬度自动降落
			 if(return_home(4,0.8,NEN_LAND,NEN_HEAD_WAY,dt))
					mission_state=7;
			 break;		 	 			
	case 13://航线飞行			
				flag=way_point_mission(1,NEN_HEAD_WAY,dt);	
        if(flag)
				{flag=0;aux_open(2,1);mission_state=4;}					
	break;			
	//--------------------
	case 102://对准伤员 达到设定高度后投递医疗包
		    mode_oldx.sdk_flag=1;//
		    flag=target_track_downward_task_search(&c2c,0.25,4.5,dt);
				if(cnt_task[2]++>30/dt)
				{mission_state=3;cnt_task[2]=flag=cnt_task[5]=0;}
				
				if(flag)//投递货物
				{mission_state=4;//to return home
				 cnt_task[2]=flag=0;
				 aux_open(2,1);
				 cnt_task[6]=1;//drop flag
				}
	      break;
				
	default:break;
	}
	

	if(flag)
		mission_state++;
	
	mission_time+=dt;
	if(mission_time>3*60&&(mission_state>=100||mission_state==3))//任务超时则自动返航
	{mission_state=13;cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=0;}
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	
```

### 7.3 自动起飞->激光雷达避障->自主返航
____该SDK是IMAV比赛中一个项目，项目要求如下：起飞前给定参赛选手5分钟前一个伤员的经纬度，无人机需要资助飞行到
该区域但与7.2不同的时目标处于密集树林内，因此需要在搜寻时能自动躲避障碍物，则使用2D激光雷达得到飞行器避障速度
进行前进搜索中不但躲避树木。

```

u8 mission_avoid(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float lat,lon;
	static float mission_time;
	pan_set_task(90,dt);
	float dis=0;
	switch(mission_state){
	case 0:
			   flag=take_off_task1(1.68,1.5,dt);//自动起飞
		    break;
	case 1:
				 pan_set_task(90,dt);
				 flag=set_drone_z_task(1.68,dt);//达到1.6m悬停
				 lat=navData.missionLegs[LIMIT(navData.Leg_num-1,0,99)].targetLat;//给定经纬度
				 lon=navData.missionLegs[LIMIT(navData.Leg_num-1,0,99)].targetLon;
				 mission_time=0;
		     break;	
	case 2:
		     mode_oldx.sdk_flag=1;//图像切换伤员识别
		     set_drone_z_task(1.5,dt);//期望仿地飞行搜索高度
			 flag=avoid_way_point_task(&c2c,lat,lon,0.25,0.2,0.5,0,dt);
			 
			 if(c2c.check)
			  cnt_task[0]++;
			 
			  if(cnt_task[0]>5){//投递货物
				  aux_open(2,1);
				  flag=1;
				}
	       break;	
	case 3:return_home(10,2,EN_LAND,NEN_HEAD_WAY,dt);//自动返航
	       break;					
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	mission_time+=dt;
	if(mission_time>7*60&&(mission_state>100||mission_state==2))//超时返航
		mission_state=3;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	
```


# 8 捐赠与项目后续开发计划
____如果您觉得该项目对您有帮助，也为了更好的项目推进和软硬件更新，如果愿意请通过微信捐赠该项目！
<div align=center><img width="440" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.png"/></div>





