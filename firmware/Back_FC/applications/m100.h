


#ifndef _M100_H
#define _M100_H

#include "stm32f4xx.h"
void m100_disarm(u16 delay);
void m100_arm(u16 delay);
void m100_activate(u16 delay);
void m100_activate_long(u8 times);
void m100_obtain_control(u16 delay);
void m100_obtain_control_long(u8 times);
void m100_dis_control(u16 delay);
void m100_dis_control_long(u8 times);
void m100_rth(u16 delay);
void m100_take_off(u16 delay);
void m100_take_off_long(u8 times);
void m100_land_control(u16 delay);
void m100_land_control_long(u8 times);
void m100_vRc_on_A(u16 delay);
void m100_vRc_on_F(u16 delay);
void m100_vRc_off(u16 delay);
void m100_contrl(float x,float y,float z,float yaw,u8 mode);
void m100_data(u16 delay);
void m100_rst(u16 delay);
void px4_control_publish(float x,float y,float z,float yaw,u8 mode,u8 save_data, u8 save_video);
void m100_contrl_px4(float x,float y,float z,float yaw,u8 mode,u8 save_data, u8 save_video);
void sdk_task(float dt);
extern u16 Rc_Pwm_Out_mine_USE[8],Rc_Pwm_Inr_mine[8],Rc_Pwm_Inr_mine[8];
extern float k_px4[4],tar_px4[4];
extern float k_px4_rc[4];
extern u8 mission_switch;
void m100_data_save_publish(void);
void send_to_px4(void);
void V_SLAM_CONTROL(void);
#define PX4_MODE_MISSION 2
#define PX4_MODE_OFFBOARD 1
#define PX4_MODE_LAND 21
#define PX4_MODE_TAKEOFF 22
#define PX4_MODE_ARM 11
#define PX4_MODE_DISARM 12
#endif

