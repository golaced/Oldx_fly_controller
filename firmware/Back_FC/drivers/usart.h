#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
extern u8 rc_board_connect, m100_connect;
extern u16 rc_board_connect_lose_cnt;
extern float Yaw_fc1;
extern u8 Rx_Buf[], pos_kf_state[3];
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
void UsartSend_GOL_LINK_NAV(uint8_t ch);
void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length);

extern u8 Rx_Buf[];
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart3_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
void APP_LINK(void);
typedef struct PID_STA{u16 OP,OI,OD,IP,II,ID,YP,YI,YD;}PID_STA;
extern PID_STA HPID,SPID,FIX_PID,NAV_PID;
extern PID_STA HPID_app,SPID_app,FIX_PID_app,NAV_PID_app;
extern u32 imu_loss_cnt,app_connect_fc_loss;
extern float POS_UKF_X,POS_UKF_Y,VEL_UKF_X,VEL_UKF_Y;
extern u8 app_connect_fc;

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
	      int16_t HEIGHT_MODE,ATT_MODE;
	      int16_t POS_MODE;
	      int16_t SWITCH_MODE[3];
				float pos_switch[4];
	      u8 update,Heart,Heart_rx,Heart_error;
	      u16 lose_cnt,lose_cnt_rx;
	      u8 connect;
				int16_t RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get,Rc_Get_PWM,Rc_Get_SBUS;//接收到的RC数据,1000~2000
				
				
struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //原始值
	   struct _float averag;  //平均值
	   struct _float histor;  //历史值
	   struct _int16 quiet;   //静态值
	   struct _float radian;  //弧度值 
          };

struct _alt{
int32_t altitude;
float altitude_f;
int32_t Temperature;
int32_t Pressure;
int Temperat;
};
					
struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
	struct _trans hmc;
	struct _alt alt;
              };

extern struct _sensor sensor;	

struct SMART{   
	struct _float pos;
	struct _float spd;
	struct _float att_rate;
	struct _float att;
  RC_GETDATA rc;
              };	
extern struct SMART smart,smart_in;
	

struct IMU_PAR{   
    float k_flow_sel;
	  float flow_set_yaw;
	  u8 flow_out_invert[2];
	  float flow_module_offset_x,flow_module_offset_y;
              };	
extern struct IMU_PAR imu_board;
							
struct _speed{   
	int altitude;
	int bmp;
	int pitch;
	int roll;
	int gps;
	int filter;
	int sonar;
	int acc;
              };

struct _altitude{   
	int bmp;
	int sonar;
	int gps;
	int acc;
	int filter;
              };
struct _angle{   
float pitch;
float roll;
float yaw;
              };

							
struct _get{   
	struct _speed speed;
	struct _altitude altitude;
	struct _angle AngE;
              };
struct _plane{   
	struct _speed speed;
	struct _altitude altitude;
	struct _get get;
              };

extern struct _plane plane;	
							
							
struct _slam{   
	 int16_t spd[5];
	 int16_t dis[5];
              };
extern struct _slam slam;	
							
							
struct _SPEED_FLOW_NAV{
float west;
float east;
float x;
float y;
float x_f;
float y_f;

};	

struct _POS_FLOW_NAV {
float	east;
float	west;
//long LAT;
//long LON;
long Weidu_Dig;
long Jingdu_Dig;
u8 flow_qual;
};
struct _POS_GPS_NAV {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
};

struct _FLOW_NAV{
struct _SPEED_FLOW_NAV speed;
struct _SPEED_FLOW_NAV speed_h;	
struct _POS_FLOW_NAV position;
	u8 rate;
};	

struct _IMU_NAV{   
struct _FLOW_NAV flow;
struct _POS_GPS_NAV gps;
	
};
extern struct _IMU_NAV imu_nav;
	

struct _FLOW_DEBUG{   
int ax,ay,az,gx,gy,gz,hx,hy,hz;
u8 en_ble_debug;
};
extern struct _FLOW_DEBUG flow_debug;


struct _PID2{
float p;
float i;
float d;
float fp;
float i_max;
float limit;
float dead;	
float dead2;	
};
struct _PID1{
struct _PID2 out;
struct _PID2 in;	
};
struct _PID_SET{
struct _PID1 nav;
struct _PID1 high;
struct _PID1 avoid;
struct _PID1 marker;
struct _PID1 circle;
};
extern struct _PID_SET pid;

typedef struct
{
u8 thr_fix;
u8 en_pid_out_pit;
u8 en_pid_out_rol;
u8 en_pid_out_yaw;
u8 en_pid_fuzzy_p;
u8 en_pid_sb_set;
u8 trig_flow_spd;
u8 return_home;
u8 trig_h_spd;
u8 px4_map;
u8 en_fuzzy_angle_pid;
u8 en_sensor_equal_flp;
u8 pit_rol_pid_out_flp;	
u8 en_pid_yaw_angle_control;	
u8 en_pid_yaw_control_by_pit_rol;	
u8 thr_add_pos;
u8 spid;
u8 mpu6050_bw_42;
u8 en_imu_q_update_mine;	
u8 en_moto_smooth;
u8 pid_mode;
u8 no_head;
u8 sonar_avoid;	
u8 yaw_sel;
u8 sb_smooth;
u8 use_px4_err;
u8 flow_hold_position;
u8 dji_sonar_high;
u8 led_flag;
u8 auto_fly_up,auto_land,en_auto_home,auto_sdk,rc_loss_return_home;
u8 en_circle_nav,circle_miss_fly,en_track_forward,en_circle_locate;
u8 flow_hold_position_use_global;
u8 flow_hold_position_high_fix;
u8 height_safe;
u8 baro_lock;
u8 baro_f_use_ukfm;
u8 flow_f_use_ukfm;
u8 use_ano_bmp_spd;
u8 tune_ctrl_angle_offset;
u8 imu_use_mid_down;
u8 hunman_pid;
u8 yaw_imu_control;	
u8 cal_sel;
u8 flow_sel;
u8 height_in_speed;
u8 height_upload;
u8 en_h_mode_switch;
u8 en_dj_cal;
u8 en_sd_save;
u8 cal_rc;
u8 en_break;
u8 use_dji;
u8 en_marker;
u8 rc_control_flow_spd;
u8 rc_control_flow_pos;
u8 rc_control_flow_pos_sel;
u8 dj_by_hand;
u8 en_dj_control;
u8 dj_yaw_follow;
u8 dji_mode;
u8 en_visual_control;
u8 hold_use_flow;
u8 en_sonar_avoid;
u8 thr_fix_test;
u8 en_imu_ekf;
u8 att_pid_tune;
u8 high_pid_tune;
u8 dj_lock;
u8 en_eso;
u8 en_eso_h_out;
u8 en_eso_h_in;
u8 yaw_use_eso;
u8 flow_d_acc;
u8 en_hinf_height_spd;
u8 en_circle_control;
u8 save_video;
u8 en_h_inf;
u8 test1;
u8 test2;
u8 test3;
u8 test4;	
u8 use_uwb_as_pos;
u8 mode_fly;
u8 h_is_fix;
u8 mems_state;
u8 att_ident1;
//flow
u8 en_flow_gro_fix;
u8 flow_size;
u8 show_qr_origin;
//test
u8 fc_test1;
u8 sdk_flag;
}_MODE;

extern _MODE mode_oldx;

typedef struct{
	unsigned int x;//目标的x坐标
	unsigned int y;//目标的y坐标
	unsigned int w;//目标的宽度
	unsigned int h;//目标的高度
	
	u8 check;
}RESULT;//识别结果

extern RESULT color;
extern u8 LOCK, KEY[8],KEY_SEL[4],NAV_BOARD_CONNECT;
extern struct _MODE mode;
extern void GOL_LINK_TASK(void);
extern void SD_LINK_TASK(void);
extern void Usart1_Init(u32 br_num);//SD_board
extern void Usart4_Init(u32 br_num);//-------SD_board
extern void Usart3_Init(u32 br_num);//-------CPU_board
extern u8 cnt_nav_board;
extern u16 data_rate_gol_link;
extern void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
extern void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void SD_LINK_TASK2(u8 sel);
extern void Send_MODE_SD(void);
void Send_IMU_NAV(void);
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
#define SEND_PID 4
#define SEND_DEBUG 5
#define SEND_QR 6
#define SEND_M100 7
#define SEND_MARKER 8
#define SEND_SD_SAVE1 9
#define SEND_SD_SAVE2 10
#define SEND_SD_SAVE3 11
#define SEND_NRF1 12
#define SEND_NRF2 13
#define SEND_NRF3 14

extern int sd_save[25*3];
void sd_publish(void);
extern float rate_gps_board;
void Send_DJI(void);
void Send_DJI2(void);
void Send_DJI3(void);
void Uart3_Send(u8 data_num);
extern u16 DJI_RC[4],DJI_RC_TEMP[4],PWM_DJI[4];
void Send_IMU_TO_GPS(void);
#define SEND_BUF_SIZE1 64+32	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
extern u8 SendBuff1_cnt;
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void data_per_uart4_ble(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);


#define SEND_BUF_SIZE2 40	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void);

#define SEND_BUF_SIZE3 32	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void);

#define SEND_BUF_SIZE4 64*3	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void data_per_uart4(u8 sel);

void UsartSend_GPS(uint8_t ch);

void Usart1_Send_DMA(u8 *dataToSend , u8 length);
extern int16_t BLE_DEBUG[16];
// pxy
extern char USART1_Flag ;
void Blocks_Receive_Data(void);
typedef struct Pixy_Color//单色块位置大小信息
{
	s16 Pixy_Sig;   //1-7 for normal signatures
	s16 Pixy_PosX;  //0 to 319
	s16 Pixy_PosY;  //0 to 199
	s16 Pixy_Width; //1 to 320
	s16 Pixy_Height;//1 to 200
	s16 Pixy_Angle; //仅在CCs颜色代码模式下才可用
}Pixy_Color;
extern Pixy_Color Pixy;//结构体定义
extern float flow_matlab_data[4],baro_matlab_data[2];

void clear_nrf_uart(void);
 typedef struct
{
 float Pit,Rol,Yaw;
 double Lat,Lon;
 float H,H_Spd,H_G;	
 float q[4];
 u8 GPS_STATUS;	//>3  5->Best
 float X_Spd_b,Y_Spd_b,X_Spd_n,Y_Spd_n;
 int Dop[6];
/*
Flight status val	status name
1	standby
2	take_off
3	in_air
4	landing
5	finish_landing
*/
 u8 STATUS;
 float Bat;	
 int Rc_pit,Rc_rol,Rc_yaw,Rc_thr,Rc_mode,Rc_gear;
 u8 m100_connect,m100_data_refresh;
 double Init_Lat,Init_Lon;
 float Y_Pos_local,X_Pos_local,spd[3];
 long r1,r2;
 float rx_dt;
 u8 control_connect,connect;
 u16 control_loss,cnt_m100_data_refresh,loss_cnt;
 u8 px4_tar_mode;
 float control_spd[3];
 float control_yaw;
 float uwb_o[3];
 u8 save_data,save_video;
 u8 navigation_mode;
 float uwb_f[4];//x y z yaw
}M100;
extern M100 m100,px4;
extern u16 nrf_uart_cnt;



struct _QR{
float x,y,z,pit,rol,yaw;
float spdx,spdy,spdz;
u8 check,use_spd,connect;
float yaw_off;
u8 insert;
u16 loss_cnt;
};
extern struct _QR qr;

struct ROBOT{
float track_x,track_y,track_r;
float camera_x,camera_y,camera_z,pit,rol,yaw,mark_x,mark_y,mark_r,mark_track_off[2];
float pos_est[3],vel_est[3],att_est[3];
float spd_robot[2];
u8 connect,mark_check;
float mark_map[6][5];
u16 loss_cnt;
};
extern struct ROBOT robot;
extern float k_dj[5];

struct _FLOW_PI{
float x,y,z,z_o,pit,rol,yaw;
float spdx,spdy,spdz;
float yaw_off;
u16 loss_cnt;
u8 check,use_spd,connect,insert;
struct _QR sensor;
float acc[3];
float gyro[3];
int mark_map[10][5];	
};
extern struct _FLOW_PI pi_flow;
extern int debug_pi_flow[20];

//car
extern float car_spd[3],car_pos[3];
typedef struct  { 
 float encoder[3];
 float body_spd[3];//x y w
 float car_cmd_spd[3];
 float pos[3],pos_off[3];
 u8 car_mode;//0 遥控   2 前进 1 往返 3 圆
 float gain[3];
	}_CAR ;
extern _CAR three_wheel_car;	
#endif
