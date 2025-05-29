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


int cmd_fd = -1;



bool init_cmd_uart(void) {
    // open the same TTY write-only; termios already configured by first open
    cmd_fd = open(UART_DEVICE, O_WRONLY | O_NOCTTY | O_SYNC);
    if (cmd_fd < 0) {
        std::cerr
          << "[MAVLink:ERROR] init_cmd_uart: open(\"/dev/serial0\"): "
          << strerror(errno)
          << "\n";
        return false;
    }
    return true;
}

void Send(int fd, const uint8_t *buf, size_t len) {
    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        perror("MAVLink:UART write failed");
    } else if ((size_t)n != len) {
        std::cerr << "[MAVLink:WARN] only wrote " << n << "/" << len << " bytes\n";
    } else if (DEBUG_SEND) {
        std::cerr << "[MAVLink] wrote " << n << " bytes:";
        for (size_t i = 0; i < (size_t)n; ++i)
            std::cerr << " " << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i];
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
    std::cout << "[MAVLink] Sent servo command: servo=" << (int)servo << ", pwm=" << pwm << "\n";
}

void clearMission() {
    mavlink_message_t msg;
    uint8_t        buf[MAVLINK_MAX_PACKET_LEN];
    size_t         len;

    mavlink_msg_mission_clear_all_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS,
        TARGET_COMP,
        MAV_MISSION_TYPE_ALL
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    pthread_mutex_lock(&shm_ptr->mutex);
    shm_ptr->seq = 0;
    pthread_mutex_unlock(&shm_ptr->mutex);
    std::cout << "[MAVLink] Sent MISSION_CLEAR_ALL\n";
}

/*
TO_DO: THis dosn't work very well
*/
bool setFlightMode(FlightMode fm) {
    // 1) pick the right MAV_MODE_* define for param1
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 10; 

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
        base_mode, // PARAM1: the full mode flags
        custom_mode,                // PARAM2: custom mode (ignored)
        0,                // PARAM3: custom submode (ignored)
        0, 0, 0, 0          // PARAM4–7: unused
    );

    // 3) send it
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t  len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[MAVLink] Sent flight mode command: " << static_cast<int>(fm) << "\n";
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
    std::cout << "[MAVLink] Sent throttle command: " << (arm ? "ARM" : "DISARM") << "\n";
}


void appendWaypoint(double lat, double lon, float alt, std::vector<MissionItem>& missionPlan) {
    int32_t lat_fixed7 = static_cast<int32_t>(std::lround(lat * 1e7));
    int32_t lon_fixed7 = static_cast<int32_t>(std::lround(lon * 1e7));

    mavlink_mission_item_int_t item{};
    item.seq = missionPlan.size();
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.current = 0;
    item.autocontinue = 1;
    item.param1 = 0.0f;  // hold time
    item.param2 = 0.0f;  // acceptance radius
    item.param3 = 0.0f;
    item.param4 = 0.0f;
    item.x = lat_fixed7;
    item.y = lon_fixed7;
    item.z = alt;
    item.mission_type = MAV_MISSION_TYPE_MISSION;

    missionPlan.push_back({item});

    std::cout << "[MAVLink] appendWaypoint: lat=" << lat_fixed7
              << ", lon=" << lon_fixed7
              << ", alt=" << alt
              << ", seq=" << item.seq
              << ", current=" << (int)item.current << "\n";
}



void appendTurn(float yaw_deg, float yaw_rate, int8_t direction, bool relative, std::vector<MissionItem>& missionPlan) {
    mavlink_mission_item_int_t item{};
    item.seq = missionPlan.size();
    item.frame = MAV_FRAME_MISSION;
    item.command = MAV_CMD_CONDITION_YAW;
    item.current = 0;
    item.autocontinue = 1;
    item.param1 = yaw_deg;
    item.param2 = yaw_rate;
    item.param3 = direction;
    item.param4 = relative ? 1 : 0;
    item.x = 0;
    item.y = 0;
    item.z = 0;
    item.mission_type = MAV_MISSION_TYPE_MISSION;

    missionPlan.push_back({item});

    std::cout << "[MAVLink] appendTurn: yaw=" << yaw_deg
              << ", rate=" << yaw_rate
              << ", direction=" << (int)direction
              << ", relative=" << relative << "\n";
}


// FSM states for mission upload handshake
void startMissionUpload(std::vector<MissionItem>& missionPlan) {
    pthread_mutex_lock(&shm_ptr->mutex);
    uint8_t current_seq = shm_ptr->current_seq;
    pthread_mutex_unlock(&shm_ptr->mutex);

    //Remove Items that have been completed
    
    if (current_seq > 0 && current_seq < missionPlan.size()) {
        missionPlan.erase(missionPlan.begin(), missionPlan.begin() + current_seq);
        for (size_t i = 0; i < missionPlan.size(); ++i) {
            missionPlan[i].msg.seq = i;
            missionPlan[i].msg.current = 0;
        }
        //reset current_seq
        pthread_mutex_lock(&shm_ptr->mutex);
        shm_ptr->current_seq = 0;
        pthread_mutex_unlock(&shm_ptr->mutex);
    }
    else if (current_seq >= missionPlan.size()) {
        std::cout << "[MAVLink] All mission items already uploaded.\n";
        return;
    }


    uint16_t totalSize = missionPlan.size();
    if (totalSize == 0) {
        std::cerr << "[MAVLink:WARN] No mission items to upload.\n";
        return;
    }

    // Clear vehicle mission storage
    mavlink_message_t clear_msg;
    uint8_t clear_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_clear_all_pack(
        SYSID, COMPID, &clear_msg,
        TARGET_SYS, TARGET_COMP,
        MAV_MISSION_TYPE_MISSION
    );
    size_t clear_len = mavlink_msg_to_send_buffer(clear_buf, &clear_msg);
    Send(cmd_fd, clear_buf, clear_len);
    std::cout << "[MAVLink] Sent MISSION_CLEAR_ALL\n";

    // Send mission count
    mavlink_message_t count_msg;
    uint8_t count_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_count_pack(
        SYSID, COMPID, &count_msg,
        TARGET_SYS, TARGET_COMP,
        totalSize,
        MAV_MISSION_TYPE_MISSION,
        0
    );
    size_t count_len = mavlink_msg_to_send_buffer(count_buf, &count_msg);
    Send(cmd_fd, count_buf, count_len);
    std::cout << "[MAVLink] Sent MISSION_COUNT: " << totalSize << " items\n";

    // FSM loop to handle upload
    size_t nextIndex = 0;
    while (true) {
        pthread_mutex_lock(&shm_ptr->mutex);
        MissionExecState state = shm_ptr->missionState;
        pthread_mutex_unlock(&shm_ptr->mutex);

        if (state == NEXT && nextIndex < missionPlan.size()) {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_mission_item_int_pack(
                SYSID, COMPID, &msg,
                TARGET_SYS, TARGET_COMP,
                missionPlan[nextIndex].msg.seq,
                missionPlan[nextIndex].msg.frame,
                missionPlan[nextIndex].msg.command,
                missionPlan[nextIndex].msg.current,
                missionPlan[nextIndex].msg.autocontinue,
                missionPlan[nextIndex].msg.param1,
                missionPlan[nextIndex].msg.param2,
                missionPlan[nextIndex].msg.param3,
                missionPlan[nextIndex].msg.param4,
                missionPlan[nextIndex].msg.x,
                missionPlan[nextIndex].msg.y,
                missionPlan[nextIndex].msg.z,
                missionPlan[nextIndex].msg.mission_type
            );

            size_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Send(cmd_fd, buf, len);

            std::cout << "[MAVLink] Sent mission item seq: " << nextIndex << "\n";
            nextIndex++;

            pthread_mutex_lock(&shm_ptr->mutex);
            shm_ptr->missionState = (nextIndex >= missionPlan.size()) ? COMPLETED : RUNNING;
            pthread_mutex_unlock(&shm_ptr->mutex);
        }

        if (state == COMPLETED) {
            pthread_mutex_lock(&shm_ptr->mutex);
            shm_ptr->missionState = IDLE;
            shm_ptr->missionCount = missionPlan.size();
            pthread_mutex_unlock(&shm_ptr->mutex);

            std::cout << "[MAVLink] Mission upload completed successfully.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}


int getMissionSize(std::vector<MissionItem>& missionPlan) {
    return missionPlan.size();
}

void setMissionCurrent(uint16_t seq) {
    if (cmd_fd < 0) {
        std::cerr << "[MAVLink] setMissionCurrent: UART not initialized\n";
        return;
    }

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_DO_SET_MISSION_CURRENT,  // command ID 226
        0,        // confirmation
        static_cast<float>(seq),  // param1: mission item index to jump to
        0, 0, 0, 0, 0, 0          // other params not used
    );

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);

    std::cout << "[MAVLink] Sent MAV_CMD_DO_SET_MISSION_CURRENT, seq = " << seq << "\n";
}


void sendMissionResumeCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_DO_PAUSE_CONTINUE,
        0,
        1,
        0, 0, 0, 0, 0, 0);

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[MAVLink] Sent mission resume\n";
}

void sendMissionPauseCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_DO_PAUSE_CONTINUE,
        0,
        0,
        0, 0, 0, 0, 0, 0);

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[MAVLink] Sent mission pause\n";
}

void sendMissionStopCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_GUIDED_DISARMED,
        0, 0, 0, 0, 0, 0);

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[MAVLink] Sent mission resume\n";
}
void sendMissionStartCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_MISSION_START,
        0,
        0,
        0, 0, 0, 0, 0, 0);

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[MAVLink] Sent mission start\n";
}

