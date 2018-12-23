#include "usart.h"
#include "mavl.h"
#include "stm32f4xx_usart.h"
#include "fifo.h"
#include "usart.h"
//
#include "imu.h"
#include "time.h"
#include "rc_mine.h"
//
navStruct_t navData;
mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;
mavlink_message_t* msg;

fifo_t uart_rx_fifo, uart_tx_fifo;
uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE], uart_rx_buf[UART_RX_BUFFER_SIZE];

void mavlink_init(void){
	mavlink_system.sysid = MAV_TYPE_FIXED_WING;
	
	mavlink_system.compid = MAV_AUTOPILOT_PIXHAWK;
	//mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
	
	mavlinkData.wpCurrent = mavlinkData.wpCount + 1;
	fifo_init(&uart_tx_fifo, uart_tx_buf, UART_TX_BUFFER_SIZE);	
	fifo_init(&uart_rx_fifo, uart_rx_buf, UART_RX_BUFFER_SIZE);
}

	
//Add By BigW
typedef uint8_t bool;
typedef struct {
    char c;
} prog_char_t;
	
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
//定义发送消息
static int8_t control_mode = STABILIZE;
mavlink_channel_t           chan;
uint16_t                    packet_drops;

mavlink_heartbeat_t         heartbeat;
mavlink_attitude_t          attitude;
mavlink_global_position_int_t position;
mavlink_ahrs_t              ahrs;

mavlink_mission_current_t current_mission;

uint8_t buf[100];
static mavlink_statustext_t pending_status;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

void handleMessage(mavlink_message_t* msg);

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */
#include "height_ctrl.h"
#include "pos_ctrl.h"
#include "rc.h"
static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

	  switch(mode_oldx.flow_hold_position){
			case 0:
			if(height_ctrl_mode!=0)
				control_mode=ALT_HOLD;
			else
				control_mode=STABILIZE;
			break;
			case 1:
			if(height_ctrl_mode!=0)
				control_mode=GUIDED;
			else
				control_mode=STABILIZE;
			break;
			case 2:
			if(height_ctrl_mode!=0)
				if(state_v!=SU_MISSION)
				control_mode=LOITER;
				else
				control_mode=AUTO;	
			else
				control_mode=STABILIZE;
			break;
			break;
		}
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (fly_ready) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_PIXHAWK,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    mavlink_msg_attitude_send(
        chan,
        ++buf[1],//millis(),
        Rol_fc/57.3,//ahrs.roll,
        Pit_fc/57.3,//ahrs.pitch,
        Yaw_fc/57.3,//ahrs.yaw,
        0,//mpu6050_fc.Gyro_deg.x,//omega.x,
        0,//mpu6050_fc.Gyro_deg.y,//omega.y,
        0);//mpu6050_fc.Gyro_deg.z);//omega.z);
}
 
static void NOINLINE send_location(mavlink_channel_t chan)
{
    //Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
    if(ABS(m100.Lat)<5||ABS(m100.Lon)<5){
			m100.Lat=39.9626679;//39.92;
		  m100.Lon=116.3040411;//114.46;
		}
	   mavlink_msg_global_position_int_send(
        chan,
        1,//millis(),
        m100.Lat*10000000,//current_loc.lat,                // in 1E7 degrees
        m100.Lon*10000000,//current_loc.lng,                // in 1E7 degrees
        drone.pos[2]*1000,//g_gps->altitude * 10,             // millimeters above sea level
        drone.pos[2]*1000,//(current_loc.alt - home.alt) * 10,           // millimeters above ground
        drone.spd[0]*100,//g_gps->ground_speed * rot.a.x,  // X speed cm/s
        drone.spd[1]*100,//g_gps->ground_speed * rot.b.x,  // Y speed cm/s
        drone.spd[2]*100,//g_gps->ground_speed * rot.c.x,
        9);//g_gps->ground_course);          // course in 1/100 degree
			mavlink_msg_local_position_ned_send(chan, 1,
  		drone.pos[1], drone.pos[0], drone.pos[2], drone.spd[1], drone.spd[0], drone.spd[2]);
}

#include "rc.h"
static void NOINLINE send_rc(mavlink_channel_t chan){
 mavlink_msg_rc_channels_raw_send(chan, 1, 0, RX_CH_PWM[2]-1000+1024, RX_CH_PWM[0]-1000+1024, 
	RX_CH_PWM[1]-1000+1024, RX_CH_PWM[3]-1000+1024,Rc_Get_PWM.AUX1-1000+1024, Rc_Get_PWM.AUX2-1000+1024, Rc_Get_PWM.AUX3-1000+1024, Rc_Get_PWM.AUX4-1000+1024, 255);
}
	
	
static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    //Vector3f omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        ++buf[8],//omega_I.x,
        ++buf[9],//omega_I.y,
        ++buf[10],//omega_I.z,
        1,
        0,
        ++buf[11],//ahrs.get_error_rp(),
        ++buf[12]);//ahrs.get_error_yaw());
}


static void NOINLINE send_statustext(mavlink_channel_t chan)
{
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    return false;
}

//发送
// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = serial_free();

    if (telemetry_delayed(chan)) {
        return false;
    }

    switch(id) {
      case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        break;

      case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

      case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;
		
      case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(chan);
        break;

			case MSG_RADIO_IN:
         send_rc(chan);
			 break;
			
      case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

		  default:
			  break;
    }
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, enum gcs_severity severity, char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        mav_array_memcpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(
            chan,
            severity,
            str);
    }
}

mavlink_message_t msg_get;
//处理接受消息
void update_mavlink(void)
{
	  u8 get_msg=0;
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    while(serial_available())
    {
        uint8_t c = serial_read_ch(); 
        get_msg=mavlink_parse_char(chan, c, &msg_get, &status);  
			  if(msg_get.msgid==0x2b)
					get_msg=2;
        // Try to get a new message
        if (get_msg) {
            mavlink_active = true;
            handleMessage(&msg_get);
        }
    }
}



unsigned int navGetWaypointCount(void) {
    int i;

    for (i = 0; i < NAV_MAX_MISSION_LEGS; i++)
        if (navData.missionLegs[i].type == 0)
            break;
    navData.Leg_num=i;
    return i;
}

unsigned char navClearWaypoints(void) {
    unsigned char ack = 0;
    int i;

    if (1){//navData.mode != NAV_STATUS_MISSION) {
        for (i = 0; i < NAV_MAX_MISSION_LEGS; i++)
            navData.missionLegs[i].type = 0;
        ack = 1;
    }

    return ack;
}

navMission_t *navGetWaypoint(int seqId) {
    return &navData.missionLegs[seqId];
}

navMission_t *navGetHomeWaypoint(void) {
    return &navData.homeLeg;
}

u16 handle_id; 
void handleMessage(mavlink_message_t* msg)
{
		if(msg->msgid!=0)
		 handle_id=msg->msgid;
    //struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: 
					  mavlink_msg_heartbeat_decode(msg, &heartbeat); 
						navGetWaypointCount();
            break;
				//-------------------------------gps------------------------------
				case MAVLINK_MSG_ID_MISSION_CURRENT: 
					  mavlink_msg_mission_current_decode(msg, &current_mission);
					  break;
				

				
				case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
				if (mavlink_msg_mission_clear_all_get_target_system(msg) == mavlink_system.sysid) {
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, navClearWaypoints());
				}
				break;

				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				if (mavlink_msg_mission_request_list_get_target_system(msg) == mavlink_system.sysid) {
				mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg->sysid, msg->compid, navGetWaypointCount());
				}
				break;
				
				case MAVLINK_MSG_ID_MISSION_REQUEST:
				if (mavlink_msg_mission_request_get_target_system(msg) == mavlink_system.sysid) {
				uint16_t seqId;
				uint16_t mavFrame;
				navMission_t *wp;

				seqId = mavlink_msg_mission_request_get_seq(msg);
				wp = navGetWaypoint(seqId);
				if (wp->relativeAlt == 1)
					mavFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
				else
					mavFrame = MAV_FRAME_GLOBAL;

				if (wp->type == NAV_LEG_HOME) {
					wp = navGetHomeWaypoint();

					mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg->sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, MAV_FRAME_GLOBAL, MAV_CMD_NAV_RETURN_TO_LAUNCH, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, 0.0f, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
				}
				else if (wp->type == NAV_LEG_GOTO) {
					mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg->sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_WAYPOINT, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, wp->maxHorizSpeed, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
				}
				else if (wp->type == NAV_LEG_TAKEOFF) {
					mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg->sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_TAKEOFF, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, wp->poiHeading, wp->maxVertSpeed, wp->targetLat, wp->targetLon, wp->targetAlt);
				}
				else if (wp->type == NAV_LEG_ORBIT) {
					mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg->sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, 1, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, wp->maxHorizSpeed, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
				}
				else if (wp->type == NAV_LEG_LAND) {
					mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg->sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_LAND, (navData.missionLeg == seqId) ? 1 : 0, 1,
				0.0f, wp->maxVertSpeed, wp->maxHorizSpeed, wp->poiAltitude, wp->targetLat, wp->targetLon, wp->targetAlt);
				}
				else {
					mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg->sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_WAYPOINT, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, 0.0f, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
				}
				}
				break;

				case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
				if (mavlink_msg_mission_count_get_target_system(msg) == mavlink_system.sysid) {
				uint16_t seqId;

				seqId = mavlink_msg_mission_set_current_get_seq(msg);
				if (seqId < NAV_MAX_MISSION_LEGS)
				 ; //navLoadLeg(seqId);
				}
				break;

				case MAVLINK_MSG_ID_MISSION_COUNT:
				if (mavlink_msg_mission_count_get_target_system(msg) == mavlink_system.sysid) {
				uint8_t count, ack;

				count = mavlink_msg_mission_count_get_count(msg);
				if (count > NAV_MAX_MISSION_LEGS || navClearWaypoints() != 1) {
					// NACK
					ack = 1;
					//AQ_PRINTF("Error: %u waypoints exceeds system maximum of %u.", count, NAV_MAX_MISSION_LEGS);
				}
				else {
					mavlinkData.wpTargetSysId = msg->sysid;
					mavlinkData.wpTargetCompId = msg->compid;
					mavlinkData.wpCount = count;
					mavlinkData.wpCurrent = mavlinkData.wpAttempt = 0;
					mavlinkData.wpNext = GetSysTime_us();
					ack = 0;
				}

				mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
				}
				break;

				case MAVLINK_MSG_ID_MISSION_ITEM:
				if (mavlink_msg_mission_item_get_target_system(msg) == mavlink_system.sysid) {
				uint16_t seqId;
				uint8_t ack = 0;
				uint8_t frame;

				seqId = mavlink_msg_mission_item_get_seq(msg);
				frame = mavlink_msg_mission_item_get_frame(msg);

				if (seqId >= NAV_MAX_MISSION_LEGS || (frame != MAV_FRAME_GLOBAL && frame != MAV_FRAME_GLOBAL_RELATIVE_ALT)) {
					// NACK
					ack = 1;
				}
				else {
					navMission_t *wp;
					uint8_t command;

					command = mavlink_msg_mission_item_get_command(msg);
					if (command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
				wp = navGetWaypoint(seqId);

				wp->type = NAV_LEG_HOME;

				wp = navGetHomeWaypoint();

				wp->type = NAV_LEG_GOTO;
			  if(mavlink_system.compid == MAV_AUTOPILOT_PIXHAWK){
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				//wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param1(msg);
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param2(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
        if (!isfinite(wp->poiHeading))wp->poiHeading=0;					
				}else{
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
				}
					}
					else if (command == MAV_CMD_NAV_WAYPOINT) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
						wp->relativeAlt = 1;
				else
						wp->relativeAlt = 0;

				wp->type = NAV_LEG_GOTO;
				if(mavlink_system.compid == MAV_AUTOPILOT_PIXHAWK){
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				//wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param1(msg);
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param2(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);	
        if (!isfinite(wp->poiHeading))wp->poiHeading=0;					
				}else{
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
				}
					}
					else if (command == MAV_CMD_DO_SET_HOME) {
				// use current location
				if (mavlink_msg_mission_item_get_current(msg)) {
						//navSetHomeCurrent();
				}
				// use given location
				else {
						wp = navGetHomeWaypoint();

						wp->type = NAV_LEG_GOTO;
					if(mavlink_system.compid == MAV_AUTOPILOT_PIXHAWK){
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				//wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param1(msg);
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param2(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);	
        if (!isfinite(wp->poiHeading))wp->poiHeading=0;						
				}else{
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
				}
				}
					}
					else if (command == MAV_CMD_NAV_TAKEOFF) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
						wp->relativeAlt = 1;
				else
						wp->relativeAlt = 0;

				wp->type = NAV_LEG_TAKEOFF;
				if(mavlink_system.compid == MAV_AUTOPILOT_PIXHAWK){
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				//wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param1(msg);
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param2(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
				if (!isfinite(wp->poiHeading))wp->poiHeading=0;					
				}else{
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
				}
					}
					else if (command == 1) { // TODO: stop using hard-coded values, use enums like intended
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
						wp->relativeAlt = 1;
				else
						wp->relativeAlt = 0;

				wp->type = NAV_LEG_ORBIT;
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
					}
					else if (command == MAV_CMD_NAV_LAND) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
						wp->relativeAlt = 1;
				else
						wp->relativeAlt = 0;

				wp->type = NAV_LEG_LAND;
			  if(mavlink_system.compid == MAV_AUTOPILOT_PIXHAWK){
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				//wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param1(msg);
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param2(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);		
				if (!isfinite(wp->poiHeading))wp->poiHeading=0;
				}else{
				wp->targetLat = mavlink_msg_mission_item_get_x(msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(msg);
				}
					}
					else {
				// NACK
				ack = 1;
					}
				}

				mavlinkData.wpCurrent = seqId + 1;
				mavlinkData.wpAttempt = 0;
				mavlinkData.wpNext = GetSysTime_us();
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
				}
				break;
				
				default:
					  break;
    }     // end switch
		
} // end handle mavlink

	
	/** @brief 写数据到串口，启动发射
  *        
  * @note 数据写入发射缓冲区后，启动发射中断，在中断程序，数据自动发出
  */
uint8_t serial_write_buf(uint8_t* buf, uint16_t length) {
	uint16_t i;
	if(length == 0) return false;
	for(i=0;i<length;i++)
		SendBuff1[SendBuff1_cnt++]=*(buf+i);
	return true;
}

/** @brief 自串口读数据 
  * @return 一字节数据
  */
uint8_t serial_read_ch(void){
	uint8_t ch;	
	fifo_read_ch(&uart_rx_fifo, &ch);	
	return ch;
}

/** @breif 检测发射缓冲区剩余字节长度 
  * @return 剩余字节长度
  */
uint16_t serial_free(void){
	return fifo_free(&uart_tx_fifo);
}

uint16_t serial_available(void){
	return fifo_used(&uart_rx_fifo);
}

		
	
	