
# 1 OLDX-开源多旋翼开发平台项目  
<div align=center><img width="600" height="130" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/logo.JPG"/></div>
<div align=center><img width="440" height="280" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/fc.jpg"/></div>
  OLDX多旋翼开发平台（OLDX-FC）是由北京理工大学自动化学院所属《北理云逸科技》团队开发的一个目前国内最完整的免费开源飞控项目，随着国内开源飞控的逐步发展如匿名、
INF、无名和ACFly飞控的陆续推出，如光流、气压计和GPS等相关算法已经逐步完善，但是相比Pixhawk等国外开源飞控平台的发展和定位仍然有发展空间。OLDX-FC于14年开始对多旋翼飞行器进行研究期间也经历过开源和借鉴的过程，为希望进一步推动国内开源飞控协作开发和
相互学习、相互分享的趋势，团队将该OLDX-FC转化为开源项目，采用自由捐赠的形式继续发展
[捐赠地址](https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.png)
。
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
  OLDX-FC是一个基于STM32F4系列单片机的多旋翼飞控平台，其采用双处理器、双IMU冗余的设计，飞行控制和组合导航分别运行与不同的单片机中基于串口DMA进行高速数据交互
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
  OLDX-FC硬件采用4层板设计，通过外部电源模块进行供电支持2S~4S电池供电，具有最大12路PWM输出4路AD信号输入，板载NRF2.4通讯芯片，预留6路串口1路CAN接口<br>
  
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
飞控已经集成基础参数调节功能在完成飞行器机型设置和固件更新后，进行传感器校准即刻起飞或者进行参数调节。<br>
(1)远程调参与波形显示<br>
飞控默认采用2.4G无线与OLDX手持遥控端进行通讯，参数调节可以直接通过遥控器实现，另外手持遥控器同时具有USB虚拟串口，
遥控器工作后连接PC机则可以使用匿名地面站进行参数调节(115200默认波特率)和参数波形显示。(注：对手持控制器来说需要在PID参数界面
等等数据全部接受完成后在进行调参，否则会出现参数误写入引起的炸鸡；对地面站同样也是建议在数据通信正常后最少读取5次PID参数避免误写入问题)
<div align=center><img width="640" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/tunning.jpg"/></div>

(2)姿态参数调节<br>
飞控安装好后首先需要进行姿态参数的调节推荐采用万向轴或烤四轴的方式固定飞行器进行参数调节，通过选择调参模式确认是单轴还是全向调参：

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
高度参数调节方式类似姿态调节，如对性能要求不高则可采用默认参数通过调整感度的方式实现快速起飞。另外也可以采用上位机
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
位置参数调节方式可采用上述调参方式其波形序列为UART_UP_LOAD_SEL=3(速度)，UART_UP_LOAD_SEL=5(位置)。
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
为提高调参速度减少户外飞行所需PC机和上位机设备,OLDX飞控设计了一个手持端遥控器，其除了能使用体感模式遥控飞行器外，还能代替PC地面站进行实时参数调节，
另外其实时回传SDK主状态机和子状态机运行状态，期望高度速度信息和航线航点信息，能实时了解当前SDK运行状况和下一时刻飞行器的目标任务，解决了飞行器自主
飞行中对其内部状态难以获取的问题。
<div align=center><img width="640" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/remote.jpg"/></div>
### 5.4.1 界面介绍
OLDX-Remote开机后会显示当前遥控器通道请保证其与飞控中一致（之前在线修改），遥控器目前具有三类界面，（1）主界面：显示飞行器高度，姿态，电压和飞行时间等常用数据（2）PID参数界面与PC端地面站对应显示飞行器内部参数（3）
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
首先将OLDX飞控沿机头方向安装在机体中心，推荐使用减震处理，(不在中心可能由于旋转效应造成而外加速度影响制动或悬停)，下图给出了典型飞行器的电调PWM
信号线顺序和螺旋桨转向，如电机转向与图中相反可以重新安装或者定义YAW_INVER为1：

<div align=center><img width="640" height="200" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/drone.jpg"/></div>

安装好电调和飞控后，将Power模块使用6P双头自锁线与飞控供电端连接，同时将Power端DC输入与飞行器整体供电输入焊接在一起(2S~4S),将螺旋桨解除，采用如下图的方式
使用download和ST-link下载器连接飞控供电：
<div align=center><img width="640" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/rmd/support_file/img_file/SWD.jpg"/></div>

在确认ST-Link驱动安装无误后首先向FC飞控单片机下载程序，下载完成后可以采用Debug模式运行或者重新上电，如系统正常则蜂鸣器会响并且LED闪烁，之后采用同样方法
向IMU导航单片机下载程序，运行后查看Watch中lis3mdl结构体(如没有请手动右键添加，同时保证菜单View下拉中Periodic Update选择)中Acc_I16、Gyro_16、Mag_Adc是否是数
如有数(传感器正常)则将电路板水平静置将Acc_CALIBRATE和Gyro_CALIBRATE分别置1，查看Acc_Offset和Gyro_Offset保存的当前传感器零偏，复位芯片后查看读书是否基本一致
验证板载Flash芯片正常。<br>
完成IMU下载和电路验证后重新Debug飞控单片机，此时开机蜂鸣器应当会发出开机音乐,同样mpu6050_fc结构体中传感器参数是否刷新，校准传感器复位芯片检测Flash中
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

另外飞控安装时最好使用黑色泡沫覆盖气压计部分并3D打印外壳，防止飞行中气压计受气流干扰！！

## 6.2 飞控配置和传感器校准
## 6.3 供电与LED状态显示
## 6.4 手动飞行
## 6.5 定高飞行
## 6.6 位置悬停
## 6.7 航线飞行
## 6.8 SDK自主飞行


# 7 进阶SDK开发说明

```
//代码块

```


# 7 捐赠与项目后续开发计划
如果您觉得该项目对您有帮助，也为了更好的项目推进和软硬件更新，如果愿意请通过微信捐赠该项目！
<div align=center><img width="240" height="300" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.png"/></div>





