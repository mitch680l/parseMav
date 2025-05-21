#include "mavlink_usart.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <iostream>

#include <mutex>
static std::mutex send_mutex;
static uint16_t mission_seq = 0;

int cmd_fd = -1; 
bool init_cmd_uart(void) {
    // open the same TTY write-only; termios already configured by first open
    cmd_fd = open("/dev/ttyAMA0", O_WRONLY | O_NOCTTY | O_SYNC);
    if (cmd_fd < 0) {
        std::cerr
          << "[ERROR] init_cmd_uart: open(\"/dev/serial0\"): "
          << strerror(errno)
          << "\n";
        return false;
    }
    return true;
}

void Send(int fd, const uint8_t *buf, size_t len) {
    if (fd < 0) {
        std::cerr << "[ERROR] Send: invalid file descriptor\n";
        return;}

    write(fd, buf, len);
}

void SendHeartbeat(void) {
    if (cmd_fd < 0) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        SYSID,         
        COMPID,           
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0,
        MAV_STATE_ACTIVE
    );

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
}

void SendServo(uint8_t servo, uint16_t pwm) {
    if (cmd_fd < 0) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_servo_output_raw_pack(
        SYSID,             
        COMPID,            
        &msg,
        0,              
        0,              
        servo,          
        pwm,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0     
    );

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
}

void appendWaypoint(int32_t lat, int32_t lon, float alt) {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;

    // 1) tell it we’re writing item [mission_seq..mission_seq]
    mavlink_msg_mission_write_partial_list_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        mission_seq, mission_seq,
        MAV_MISSION_TYPE_MISSION
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    // 2) send the mission item itself
    mavlink_msg_mission_item_int_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS,
        TARGET_COMP,
        mission_seq,                        // seq
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  // frame
        MAV_CMD_NAV_WAYPOINT,               // command
        0, 1,                               // current=0, autocontinue=1
        0, 0, 0, 0,                         // params 1–4 unused here
        lat, lon, alt,                      // x/y/z
        MAV_MISSION_TYPE_MISSION
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);


    mission_seq++;
}

void appendTurn(float yaw_deg, float yaw_rate = 30.0f, int8_t direction = 1, bool relative = false) {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;
    uint8_t        rel_flag = relative ? 1 : 0;

    mavlink_msg_mission_write_partial_list_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        mission_seq, mission_seq,
        MAV_MISSION_TYPE_MISSION
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    mavlink_msg_mission_item_int_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS,
        TARGET_COMP,
        mission_seq,           // seq
        MAV_FRAME_MISSION,     // yaw has no lat/lon
        MAV_CMD_CONDITION_YAW, // command
        0, 1,                  // current=0, autocontinue=1
        yaw_deg,               // param1 = heading (deg)
        yaw_rate,              // param2 = yaw speed (deg/s)
        direction,             // param3 = 1=CW, -1=CCW
        rel_flag,              // param4 = relative? (1=yes)
        0, 0, 0,               // x/y/z unused
        MAV_MISSION_TYPE_MISSION
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    mission_seq++;
}

void clearMission() {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;

    mavlink_msg_mission_clear_all_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS,
        TARGET_COMP,
        MAV_MISSION_TYPE_MISSION
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    mission_seq = 0;
}

void startMission() {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;

    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_MISSION_START, 0,    // confirmation
        /* first_seq */ 0, /* last_seq */ 0,
        0, 0, 0, 0, 0
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
}

void switchMode() {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;
    uint8_t  base_mode   = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
    uint32_t custom_mode = 3;

    mavlink_msg_set_mode_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS,
        base_mode,
        custom_mode
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
}

/*
0	MAV_MODE_PREFLIGHT	System is not ready to fly, booting, calibrating, etc. No flag is set.
64	MAV_MODE_MANUAL_DISARMED	System is allowed to be active, under manual (RC) control, no stabilization
66	MAV_MODE_TEST_DISARMED	UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
80	MAV_MODE_STABILIZE_DISARMED	System is allowed to be active, under assisted RC control.
88	MAV_MODE_GUIDED_DISARMED	System is allowed to be active, under autonomous control, manual setpoint
92	MAV_MODE_AUTO_DISARMED	System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
192	MAV_MODE_MANUAL_ARMED	System is allowed to be active, under manual (RC) control, no stabilization
194	MAV_MODE_TEST_ARMED	UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
208	MAV_MODE_STABILIZE_ARMED	System is allowed to be active, under assisted RC control.
216	MAV_MODE_GUIDED_ARMED	System is allowed to be active, under autonomous control, manual setpoint
220	MAV_MODE_AUTO_ARMED	System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
*/
bool setFlightMode(FlightMode fm) {
    // 1) pick the right MAV_MODE_* define for param1
    uint8_t mav_mode = 0;
    switch (fm) {
      case FlightMode::MANUAL:    mav_mode = MAV_MODE_MANUAL_ARMED;    break;  // 192
      case FlightMode::STABILIZE: mav_mode = MAV_MODE_STABILIZE_ARMED; break;  // 208
      case FlightMode::ACRO:      mav_mode = MAV_MODE_MANUAL_ARMED;    break;  // fallback to manual
      case FlightMode::GUIDED:    mav_mode = MAV_MODE_GUIDED_ARMED;    break;  // 216
      case FlightMode::AUTO:      mav_mode = MAV_MODE_AUTO_ARMED;      break;  // 220
      case FlightMode::RTL:       mav_mode = MAV_MODE_AUTO_ARMED;      break;  // uses auto flag
      case FlightMode::LOITER:    mav_mode = MAV_MODE_AUTO_ARMED;      break;  // uses auto flag
    }

    // 2) pack COMMAND_LONG for MAV_CMD_DO_SET_MODE (176)
    //    PARAM1 = mav_mode (one of the MAV_MODE_*_ARMED constants)
    //    PARAM2 = Custom Mode (unused when using MAV_MODE defines)
    //    PARAM3 = Custom Submode (unused)
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_DO_SET_MODE,  // 176
        0,                    // confirmation
        static_cast<float>(mav_mode), // PARAM1: the full mode flags
        0.0f,                // PARAM2: custom mode (ignored)
        0.0f,                // PARAM3: custom submode (ignored)
        0, 0, 0, 0          // PARAM4–7: unused
    );

    // 3) send it
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t  len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    return true;
}


void throttle(int arm) {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;
    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_COMPONENT_ARM_DISARM,  // command
        0,                              // confirmation
        static_cast<float>(arm),21196.f,0,0,0,0,0                  // param1=1 → arm; rest unused
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
}

