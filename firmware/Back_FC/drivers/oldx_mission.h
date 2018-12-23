#ifndef _OLDX_MISSION_H
#define	_OLDX_MISSION_H
#include "include.h"

#define EN_HEAD_WAY 1
#define NEN_HEAD_WAY 0

#define EN_LAND 1
#define NEN_LAND 0
extern float lat_human,lon_human;
u8 mission_test4(float dt);
u8 mission_test5(float dt);
u8 mission_test8(float dt);
u8 mission_test_gps(float dt);
u8 mission_light_draw(float dt);
u8 mission_landing(float dt);
u8 mission_landing1(float dt);
u8 mission_search(float dt);
u8 mission_avoid(float dt);
#endif