#include "mavlink_usart.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <thread>



bool init_cmd_uart(void) {
    // open the same TTY write-only; termios already configured by first open
    cmd_fd = open(UART_DEVICE, O_WRONLY | O_NOCTTY | O_SYNC);
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
    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        perror("UART write failed");
    } else if ((size_t)n != len) {
        std::cerr << "[WARN] only wrote " << n << "/" << len << " bytes\n";
    } else {
        std::cerr << "[DEBUG] wrote " << n << " bytes:";
        for (size_t i = 0; i < (size_t)n; ++i)
            std::cerr << " " << std::hex << std::setw(2) << std::setfill('0')
                      << (int)buf[i];
        std::cerr << std::dec << "\n";
    }
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

    pthread_mutex_lock(&shm_ptr->mutex);
    shm_ptr->seq = 0;
    pthread_mutex_unlock(&shm_ptr->mutex);
    
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


// Queue a waypoint for upload
void appendWaypoint(double lat, double lon, float alt) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int32_t lat_fixed7 = static_cast<int32_t>(std::lround(lat * 1e7));
    int32_t lon_fixed7 = static_cast<int32_t>(std::lround(lon * 1e7));

    mavlink_msg_mission_item_int_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        missionPlan.size(),                // seq
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_WAYPOINT,
        0, 1, 0,0,0,0,
        lat_fixed7, lon_fixed7, alt,
        MAV_MISSION_TYPE_MISSION
    );
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    missionPlan.push_back({ByteBuffer(buf, buf + len)});
}

// Queue a yaw-turn command for upload
void appendTurn(float yaw_deg, float yaw_rate = 30.0f,
                int8_t direction = 1, bool relative = false) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t rel_flag = relative ? 1 : 0;

    mavlink_msg_mission_item_int_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        missionPlan.size(),                // seq
        MAV_FRAME_MISSION,
        MAV_CMD_CONDITION_YAW,
        0, 1,
        yaw_deg, yaw_rate, direction, rel_flag,
        0,0,0,
        MAV_MISSION_TYPE_MISSION
    );
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    missionPlan.push_back({ByteBuffer(buf, buf + len)});
}

// FSM states for mission upload handshake


// Send MISSION_COUNT and enter upload loop
void startMissionUpload() {
    // 1) Send total count
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
     mavlink_msg_mission_count_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        missionPlan.size(),                     // total items
        MAV_MISSION_TYPE_MISSION,               // mission type
        0                                       // timeout (ms)
    );
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[DEBUG] Sent MISSION_COUNT: " << missionPlan.size() << " items\n";

    // 2) Enter FSM to handle requests
    size_t nextIndex = 0;
    MissionUploadState state = UP_RUNNING;

    while (true) {
        // Read shared state atomically
        pthread_mutex_lock(&shm_ptr->mutex);
        auto fsm = shm_ptr->missionState;
        pthread_mutex_unlock(&shm_ptr->mutex);

        if (fsm == NEXT && state == UP_RUNNING) {
            // FC has requested next item
            if (nextIndex < missionPlan.size()) {
                Send(cmd_fd,
                     missionPlan[nextIndex].buf.data(),
                     missionPlan[nextIndex].buf.size());
                std::cout << "[DEBUG] Sent mission item seq: " << nextIndex << "\n";
                nextIndex++;
                
                pthread_mutex_lock(&shm_ptr->mutex);
                if (nextIndex >= missionPlan.size()) {
                    shm_ptr->missionState = COMPLETED;
                } else {
                    shm_ptr->missionState = RUNNING;
                }
                pthread_mutex_unlock(&shm_ptr->mutex);
            }
        }

        if (fsm == COMPLETED) {
            // Upload done
            missionPlan.clear();
            pthread_mutex_lock(&shm_ptr->mutex);
            shm_ptr->missionState = IDLE;
            pthread_mutex_unlock(&shm_ptr->mutex);
            std::cout << "[DEBUG] Mission upload completed, state reset to IDLE\n";
            break;
        }

        // Throttle loop
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Send the requested mission item (called externally if needed)
void sendMissionItem(size_t seq) {
    if (seq >= missionPlan.size()) {
        std::cerr << "[ERROR] Requested seq " << seq << " out of range\n";
        return;
    }
    Send(cmd_fd,
         missionPlan[seq].buf.data(),
         missionPlan[seq].buf.size());
    std::cout << "[DEBUG] Sent mission item seq: " << seq << "\n";
}

