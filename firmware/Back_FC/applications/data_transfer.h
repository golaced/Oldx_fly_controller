#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"
#include "height_ctrl.h"


typedef struct
{
    u8 msg_id;
    u8 msg_data;
    u8 send_check;
    u8 send_version;
    u8 send_status;
    u8 send_senser;
    u8 send_senser2;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_pid5;
    u8 send_pid6;
    u8 send_rcdata;
    u8 send_offset;
    u8 send_motopwm;
    u8 send_power;
    u8 send_user;
    u8 send_speed;
    u8 send_location;
    u8 send_qr;
} dt_flag_t;

extern dt_flag_t f;
extern u8 UART_UP_LOAD_SEL_FORCE, acc_3d_calibrate_f,acc_3d_step;;

void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_User(void);
void ANO_DT_Send_Speed(float,float,float);
void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle);
void ANO_DT_Send_QR1(float x,float y,float z);
void ANO_DT_Send_QR2(float x,float y,float z);

//HT
void HT_DT_Send_ATT(float pit,float rol,float yaw,float h);
void HT_DT_Send_HUD(u8 fly_mode,u8 lock,u8 rc_mode,float bat,float fly_speed);
void HT_DT_Send_RC(u16 rc1,u16 rc2,u16 rc3,u16 rc4,u16 rc5,u16 rc6,u16 rc7,u16 rc8,u16 rc9,u16 rc10);
void HT_DT_Send_GPS(float lon,float lat,u8 star_num,u8 latw,u8 lonw);
void HT_DT_Send_MOTOR(u16 m1,u16 m2,u16 m3,u16 m4,u16 m5,u16 m6,u16 m7,u16 m8);
void HT_DT_Send_SONAR(float l,float f,float b,float d,float r);
void HT_DT_Send_SENSOR(int ax,int ay,int az,int gx,int gy,int gz,int hx,int hy,int hz,int spdx,int spdy,int spdz,int sonar,int bmp);
#endif

