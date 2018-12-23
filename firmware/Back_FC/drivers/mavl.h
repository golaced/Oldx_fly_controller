#ifndef _MAVL_H
#define _MAVL_H
#include "stm32f4xx.h"
#include "stdint.h"
#include "mavlink_helpers.h"
#include "stm32f4xx_usart.h"
#include "fifo.h"
#include "stdio.h"
#include "minimal.h"
#include "define.h"
#define UART_TX_BUFFER_SIZE        255*2
#define UART_RX_BUFFER_SIZE        255*2

#define AQMAVLINK_WP_TIMEOUT			0.02*1000000//1e6f		    // 1 second - retry frequency for waypoint requests to planner
#define AQMAVLINK_WP_MAX_ATTEMPTS		30		    // maximum number of retries for wpnt. requests
unsigned int navGetWaypointCount(void) ;
typedef struct {
    unsigned long interval;	    // how often to send stream in us (zero to disable)
    unsigned long dfltInterval;	    // default stream interval at startup
    unsigned long next;		    // when to send next stream data
    uint8_t enable;		    // enable/disable stream
} mavlinkStreams_t;

typedef struct {
    mavlink_status_t mavlinkStatus;
    unsigned long nextHeartbeat;
    unsigned long nextParam;
    unsigned int currentParam;

    uint16_t packetDrops;	// global packet drop counter
    uint16_t idlePercent;	// MCU idle time
    unsigned long lastCounter;	// used to calculate idle time
    uint8_t indexPort;		// current port # in channels/servo outputs sequence

    // waypoint programming from mission planner
    uint8_t wpTargetSysId;
    uint8_t wpTargetCompId;
    uint8_t wpCount;		// total waypoints to expect from planner after mission_count msg
    uint8_t wpCurrent;		// current wpt sequence # requested/expected from planner
    uint32_t wpNext;		// when to send the next wpt request to planner
    uint8_t wpAttempt;		// count of consecutive wpt requests for same sequence #

} mavlinkStruct_t;

extern mavlinkStruct_t mavlinkData;


extern fifo_t uart_rx_fifo, uart_tx_fifo;
void serial_open(uint8_t port, uint32_t baud);
uint8_t serial_write_buf(uint8_t* buf, uint16_t length);
uint8_t serial_read_ch(void);
uint16_t serial_free(void);
uint16_t serial_available(void);
void mavlink_init(void);
void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
void update_mavlink(void);
void handleMessage(mavlink_message_t* msg);



#define NAV_MAX_MISSION_LEGS	50
enum navLegTypes {
    NAV_LEG_HOME = 1,
    NAV_LEG_TAKEOFF,
    NAV_LEG_GOTO,
    NAV_LEG_ORBIT,
    NAV_LEG_LAND,
    NAV_NUM_LEG_TYPES
};
typedef struct {
    double targetLat;
    double targetLon;
    float targetAlt;			// either relative or absolute - if absolute, GPS altitude is used
    float targetRadius;			// achievement threshold for GOTO or orbit radius for ORBIT
    float loiterTime;		// us
    float maxHorizSpeed;		// m/s
    float maxVertSpeed;			// m/s
    float poiHeading;			// POI heading (>= 0 is absolute, < 0 is relative to target bearing)
    float poiAltitude;			// altitude of POI - used for camera tilting
    uint8_t relativeAlt;		// 0 == absolute, 1 == relative
    uint8_t type;
} navMission_t;

typedef struct {
    navMission_t missionLegs[NAV_MAX_MISSION_LEGS];
    navMission_t homeLeg;
    uint8_t missionLeg,Leg_num;  
} navStruct_t;

extern navStruct_t navData;

#endif