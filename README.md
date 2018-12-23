# OLDX-开源多旋翼开发平台项目
<br><br>
# 1 项目介绍
<div align=center><img width="540" height="380" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/fc.jpg"/></div>
  OLDX多旋翼开发平台（OLDX-FC）是由北京理工大学自动化学院所属极客团队开发的一个目前国内最完整的免费开源飞控项目，随着国内开源飞控的逐步发展如匿名、</div>
INF、无名和ACFly飞控的陆续推出，如光流、气压计和GPS等相关算法已经逐步完善，但是相比Pixhawk等国外开源飞控平台的发展和定位仍然太过局限很多团队仍采用</div>
代码拷贝淘宝二次销售的形式导致经济竞争而非技术竞争。OLDX-FC于14年开始对多旋翼飞行器进行研究期间也经历过开源和借鉴的过程，为希望进一步推动国内开源飞控协作开发和</div>
相互学习、相互分享的趋势，团队将该OLDX-FC转化为开源项目采用自由捐赠的形式。项目遵循GPL协议，能自由下载项目PCB进行加工使用但请勿作为商业用途，开源所有</div>
飞行控制和组合导航源码，可以进行修改和二次开发。

# 2 基本功能介绍
  OLDX-FC是一个基于STM32F4系列单片机的多旋翼飞控平台，其采用双处理器、双IMU冗余的设计，飞行控制和组合导航分别运行与不同的单片机中基于串口DMA进行高速数据交互</div>
板载两套6轴惯性传感器、1个三轴磁力计、1个气压计并支持外部罗盘接入。组合导航CPU采用UCosII嵌入式操作系统基于卡尔曼滤波算法实现对GPS、光流、UWB和气压计数据的可靠</div>
融合，从而实现室内外可靠的悬停和航线飞行，姿态和高度控制采用自抗扰（ADRC）控制算法实现对外部扰动的可靠控制同时具有响应快、信号跟踪性能好的特点。飞控在内部封装</div>
SDK二次开发接口和部分Demo，能快速实现一键起飞降落，视觉降落，目标跟踪和自主避障，另外预留多个扩接口能作为地面机器人、无人车和无人船的硬件载体。

**飞控特性**
	*UCosII操作系统
	*自抗扰姿态控制
	*卡尔曼组合导航  
	*SDK快速开发
	*Mavlink航向规划和匿名地面站快速调参
	*移动遥控端状态显示和参数在线修改
	*视觉导航、自动降落、光流图像定位

***飞控性能测试视频连接：***<br>
	[SDK开发演示](https://v.youku.com/v_show/id_XMzc4MzAyMDU4MA==.html?spm=a2hzp.8244740.0.0&f=49551028) <br>
	[室外GPS航线测绘和地面站航点设置](https://i.youku.com/i/UMTU1NTgzMzYw?spm=a2hzp.8253869.0.0) <br>
	[视觉固定目标降落](https://v.youku.com/v_show/id_XMzk2OTMyNzE4MA==.html?spm=a2hzp.8253869.0.0) <br>
	[视觉移动小车降落](https://v.youku.com/v_show/id_XMzc0MTI4NzQ2NA==.html?spm=a2hzp.8253869.0.0) <br>
	[室内二维码地标阵列定位](https://v.youku.com/v_show/id_XMjc2MDAxMDI1Mg==.html?spm=a2hzp.8253869.0.0) <br>
	[室外光流悬停](https://v.youku.com/v_show/id_XMjc1NzI1NDQ4NA==.html?spm=a2hzp.8253869.0.0) <br>
	[室内气压计定高](https://v.youku.com/v_show/id_XMjcyNjM2MzU5Mg==.html?spm=a2hzp.8253869.0.0) <br>

# 3 PCB硬件参数


|列名1|列名2|
|:---|:---|
|列1的内容1|列2的内容1|
|列1的内容2|列2的内容2|

# 4 软件说明


# 5 飞控使用教程
## 5.1 PCB接口说明

## 5.2 Keil软件配置说明

## 5.3 控制参数调整和飞行器配置说明


## 5.4 首次飞行说明


# 6 进阶SDK开发说明

```
//代码块
int a=0;
a++;
```


# 7 捐赠与项目后续开发计划
如果您觉得该项目对您有帮助，也为了更好的更好的服务社区和不断更新的设计，请通过微信捐赠该项目！
<div align=center><img width="240" height="240" src="https://github.com/golaced/Oldx_fly_controller/blob/master/support_file/img_file/pay.jpg"/></div>





