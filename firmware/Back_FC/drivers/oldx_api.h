#ifndef _OLDX_API_H
#define	_OLDX_API_H
#include "pos_ctrl.h"
#include "usart.h"
#define  MAX_CNT_NUM 35
extern u32 cnt_mission[MAX_CNT_NUM];extern u16 cnt_task[MAX_CNT_NUM];
#define COMMON_INT 0
#define COMMON_CNT 1
#define COMMON_CNT1 2
#define COMMON_CNT2 3
#define SPD_MOVE_CNT 6
#define BODY_MOVE_INIT 7
#define BODY_MOVE_CNT 8
#define DRONE_Z_CNT 9
#define EST_INT 10
#define F_YAW_CNT 11
#define F_YAW_INIT 12
#define MISSION_CNT 13
#define PAN_SEARCH_FLAG 14
#define HOME_CNT 15
#define WAY_INIT 16
#define WAY_CNT 17
#define SET_POS_CNT 18
#define C_SEARCH_INT 19
#define D_DOWN_CNT 20
#define D_DOWN_INT 21
#define F_DOWN_CNT 22
#define F_DOWN_INT 23
#define DELAY_CNT 24
#define LAND_CNT 25
#define LAND_CNT1 26
#define LIGHT_DRAW_CNT1 27
#define LIGHT_DRAW_CNT2 28
#define LIGHT_DRAW_CNT3 29
#define C_SEARCH_T_INT 30
#define C_SEARCH_T_INT1 31
//-----------MODE
#define MODE_FAST_WAY 1
#define NMODE_FAST_WAY 0
#define MODE_SPD 1
#define MODE_POS 0
//-----------------------------------
#define LAND_SPD 0.68//m/s
#define MAX_WAYP_Z 100//m
#define WAY_POINT_DEAD1 0.5 //m
#define LAND_CHECK_G 0.25//g
#define LAND_CHECK_TIME 0.86//s
#define YAW_LIMIT_RATE 10//°
#define DRAW_TIME 5//s
//api
extern u32 cnt_mission[MAX_CNT_NUM];
void init_mission_reg(void);
void track_init(CI tracker);
u8 way_point_task(double lat,double lon, float height,float loter_time,u8 mode,float dt);

u8 spd_move_task(float x,float y,float z,float time,float dt);

u8 pos_move_global_task(float x,float y,float z,float dt);

u8 pos_move_body_task(float x,float y,float z,float dt);

//返航： 输入（返航高度，使能航向）
u8 back_home_task(float z,u8 yaw_target,float dt);

// 云台设置API：输入（俯仰轴：输入0则复位云台）
u8 pan_set_task(float pitch,float dt);

//云台搜索： 输入（最小角度，最大角度，递增角度，航向搜索）
void pan_search_task(float min,float max,float da,u8 en_yaw,u8 en_forward,float dt);

//降落垂直搜索：  输入（半径，高度，递增角度，递增半径）
void down_ward_search_task(float r,float z,float d_angle,float d_r,float dt);

//下置摄像头对准API： 输入（目标，下降速度，最终高度，模式）
//mode 0->位置 1->图像 2 auto
u8 target_track_downward_task(CI *target,float spdz,float end_z,u8 mode,float dt);
u8 target_track_downward_task_search(CI *target,float spdz,float end_z,float dt);
// 云台跟踪API： 输入（目标，使能逼近，使能俯仰轴，使能航向，逼近模式（0速度 1位置））
u8 target_track_pan_task(CI* target,float close_param,u8 en_pitch,u8 en_yaw,u8 close_mode,float dt);
u8 set_drone_yaw_task(float yaw,float rate_limit,float dt);//设置飞机航向
u8 set_drone_z_task(float z,float dt);//设置飞机高度
u8 target_track_forward_task(float dt);
u8 take_off_task(u8 auto_diarm,float dt);
u8 land_task(float dt);
u8 delay_task(float delay,float dt);
u8 set_pos_task(float x,float y,float z,float dt);
float est_target_state(CI* target,_CAR *target_b, float dt);
u8 traj_init_task(float ps[3],float pe[3],float T,u8 sel);
u8 follow_traj(float yaw,u8 sel,u8 mode,float dt);
u8 land_task1(float spd,float dt);	

u8 way_point_mission(float spd_limit,u8 head_to_waypoint,float dt);
u8 return_home(float set_z, float spd_limit, u8 en_land, u8 head_to_home, float dt);
u8 take_off_task1(float set_z,float set_spd,float dt);
void aux_open(u8 sel,u8 open);
u8 avoid_task(CI* target,float set_x,float set_y,float set_z, float dead, float max_spd, float dt);
u8 down_ward_search_task_tangle(float w,float h,int wn,int hn,float spd_limit,float z,float dt);
// light draw simple
u8 draw_heart(float time,float size, float dt);
u8 avoid_way_point_task(CI* target,float tar_lat,float tar_lon,float spd, float dead, float max_spd,u8 mode, float dt);
#endif