/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright ?2011-2014  Bill Nesbitt
*/
#include "includes.h"


#define GPS_STACK_SIZE          200
#define GPS_PRIORITY            35

#define GPS_BAUD_RATE           230400

#define GPS_LATENCY             75000       // us (comment out to use uBlox timepulse)
#define FLOW_PI_LATENCY         GPS_LATENCY
//#define GPS_LOG_BUF             2048        // comment out to disable logging
//#define GPS_FNAME               "GPS"
//#define GPS_DO_RTK                          // comment out to disable GPS Raw data reports
//#define GPS_DEBUG                           // uncomment to enable extra GPS messages

typedef struct {
//    OS_TID gpsTask;
//    OS_FlagID gpsVelFlag;
//    OS_FlagID gpsPosFlag;

//    serialPort_t *gpsPort;
    unsigned int baudCycle[7];
    int8_t baudSlot;
    uint8_t logHandle;

 //   digitalPin *gpsEnable;

    unsigned long iTOW;
    double lat;
    double lon;
    float height;   // above mean sea level (m)
    float hAcc;     // horizontal accuracy est (m)
    float vAcc;     // vertical accuracy est (m)
    float velN;     // north velocity (m/s)
    float velE;     // east velocity (m/s)
    float velD;     // down velocity (m/s)
    float speed;    // ground speed (m/s)
    float heading;  // deg
    float sAcc;     // speed accuracy est (m/s)
    float cAcc;     // course accuracy est (deg)
    float pDOP;     // position Dilution of Precision
    float hDOP;
    float vDOP;
    float tDOP;
    float nDOP;
    float eDOP;
    float gDOP;

    unsigned long TPtowMS;    // timepulse time of week (ms)
    unsigned long lastReceivedTPtowMS;

    unsigned long lastTimepulse;
    unsigned long lastPosUpdate;
    unsigned long lastVelUpdate;
    unsigned long lastMessage;
    signed long microsPerSecond;
} gpsStruct_t;

extern gpsStruct_t gpsData;

extern void gpsInit(void);
extern void gpsSendPacket(unsigned char len, char *buf);



#ifndef ublox_h
#define ublox_h

#define UBLOX_SYNC1	    0xB5
#define UBLOX_SYNC2	    0x62

#define UBLOX_NAV_CLASS	    0x01
#define UBLOX_RXM_CLASS	    0x02
#define UBLOX_CFG_CLASS	    0x06
#define UBLOX_MON_CLASS	    0x0a
#define UBLOX_AID_CLASS	    0x0b
#define UBLOX_TIM_CLASS	    0x0d


#define UBLOX_NAV_POSLLH    0x02
#define UBLOX_NAV_DOP	    0x04
#define UBLOX_NAV_VALNED    0x12
#define UBLOX_NAV_TIMEUTC   0x21
#define UBLOX_NAV_SBAS	    0x32
#define UBLOX_NAV_SVINFO    0x30

#define UBLOX_AID_REQ	    0x00

#define UBLOX_RXM_RAW	    0x10
#define UBLOX_RXM_SFRB	    0x11

#define UBLOX_MON_VER	    0x04
#define UBLOX_MON_HW	    0x09

#define UBLOX_TIM_TP	    0x01

#define UBLOX_CFG_PRT       0x00
#define UBLOX_CFG_MSG	    0x01
#define UBLOX_CFG_TP	    0x07
#define UBLOX_CFG_RTATE	    0x08
#define UBLOX_CFG_SBAS	    0x16
#define UBLOX_CFG_NAV5	    0x24

#define UBLOX_SBAS_AUTO	    0x00000000
#define UBLOX_SBAS_WAAS	    0x0004E004
#define UBLOX_SBAS_EGNOS    0x00000851
#define UBLOX_SBAS_MSAS	    0x00020200
#define UBLOX_SBAS_GAGAN    0x00000108

#define UBLOX_MAX_PAYLOAD   512
#define UBLOX_WAIT_MS	    20

enum ubloxStates {
    UBLOX_WAIT_SYNC1 = 0,
    UBLOX_WAIT_SYNC2,
    UBLOX_WAIT_CLASS,
    UBLOX_WAIT_ID,
    UBLOX_WAIT_LEN1,
    UBLOX_WAIT_LEN2,
    UBLOX_PAYLOAD,
    UBLOX_CHECK1,
    UBLOX_CHECK2
};

// Geodetic Position Solution
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    signed long lon;	    // Longitude (deg * 1e-7)
    signed long lat;	    // Latitude (deg * 1e-7)
    signed long height;	    // Height above Ellipsoid (mm)
    signed long hMSL;	    // Height above mean sea level (mm)
    unsigned long hAcc;	    // Horizontal Accuracy Estimate (mm)
    unsigned long vAcc;	    // Vertical Accuracy Estimate (mm)
} __attribute__((packed)) ubloxStructPOSLLH_t;

// Dilution of precision
typedef struct {
    unsigned long iTOW;	    // ms GPS Millisecond Time of Week
    unsigned short gDOP;    // Geometric DOP
    unsigned short pDOP;    // Position DOP
    unsigned short tDOP;    // Time DOP
    unsigned short vDOP;    // Vertical DOP
    unsigned short hDOP;    // Horizontal DOP
    unsigned short nDOP;    // Northing DOP
    unsigned short eDOP;    // Easting DOP
} __attribute__((packed)) ubloxStructDOP_t;

// Velocity Solution in NED
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    signed long velN;	    // NED north velocity (cm/s)
    signed long velE;	    // NED east velocity (cm/s)
    signed long velD;	    // NED down velocity (cm/s)
    unsigned long speed;    // Speed (3-D) (cm/s)
    unsigned long gSpeed;   // Ground Speed (2-D) (cm/s)
    signed long heading;    // Heading 2-D (deg * 1e-5)
    unsigned long sAcc;	    // Speed Accuracy Estimate (cm/s)
    unsigned long cAcc;	    // Course / Heading Accuracy Estimate (deg * 1e-5)
} __attribute__((packed)) ubloxStructVALNED_t;

// Timepulse Timedata
typedef struct {
    unsigned long towMS;
    unsigned long towSubMS;
    signed long qErr;
    unsigned short week;
    unsigned char flags;
    unsigned char res;
} __attribute__((packed)) ubloxStructTP_t;

// UTC Time Solution
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    unsigned long tAcc;	    // Time Accuracy Estimate
    long nano;		    // Nanosecond of second (UTC)
    unsigned short year;    // Year, range 1999..2099 (UTC)
    unsigned char month;    // Month, range 1..12 (UTC)
    unsigned char day;	    // Day of Month, range 1..31 (UTC)
    unsigned char hour;	    // Hour of Day, range 0..23 (UTC)
    unsigned char min;	    // Minute of Hour, range 0..59 (UTC)
    unsigned char sec;	    // Second of Minute, range 0..59 (UTC)
    unsigned char valid;    // Validity Flags
} __attribute__((packed)) ubloxStructTIMEUTC_t;

// Receiver/Software Version
typedef struct {
    char swVersion[30];
    char hwVersion[10];
    char extension[30][7];
} __attribute__((packed)) ubloxStructVER_t;

// Position Velocity Time Solution
typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t reserved1;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
    uint16_t reserved2;
    uint32_t reserved3;
} __attribute__((packed)) ubloxStructPVT_t;

typedef struct {
    int hwVer;

    signed long lastLat, lastLon;
    union {
	ubloxStructPOSLLH_t posllh;
	ubloxStructVALNED_t valned;
	ubloxStructDOP_t dop;
	ubloxStructTP_t tp;
	ubloxStructTIMEUTC_t timeutc;
	ubloxStructVER_t ver;
	ubloxStructPVT_t pvt;
	char other[UBLOX_MAX_PAYLOAD];
    } payload;

    unsigned char state;
    unsigned int count;
    unsigned char class;
    unsigned char id;
    unsigned int length;
    unsigned int checksumErrors;

    unsigned char ubloxRxCK_A;
    unsigned char ubloxRxCK_B;

    unsigned char ubloxTxCK_A;
    unsigned char ubloxTxCK_B;
} ubloxStruct_t;

extern unsigned char ubloxCharIn(unsigned char c);
extern void ubloxInit(void);
extern void ubloxSendSetup(void);
extern void ubloxInitGps(void);

#endif
