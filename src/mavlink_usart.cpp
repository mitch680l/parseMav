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



int uart_fd = -1;
int cmd_fd = -1;  // File descriptor for command UART
int transport_fd;


bool init_cmd_uart(void) {
#if SIM_MODE
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("[TCP_MODE] socket() failed");
        return false;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5770);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[TCP_MODE] bind() failed");
        close(sock);
        return false;
    }

    if (listen(sock, 1) < 0) {
        perror("[TCP_MODE] listen() failed");
        close(sock);
        return false;
    }

    std::cout << "[TCP_MODE] Waiting for Mission Planner to connect on port 5760...\n";
    cmd_fd = accept(sock, nullptr, nullptr);
    if (cmd_fd < 0) {
        perror("[TCP_MODE] accept() failed");
        return false;
    }

    uart_fd = cmd_fd;  // Optional: unify usage
    std::cout << "[TCP_MODE] Connected.\n";
    return true;
#else
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
#endif
}

void Send(int fd, const uint8_t *buf, size_t len) {
    #if SIM_MODE
    
    ssize_t n = write(uart_fd, buf, len);
    #else
    ssize_t n = write(fd, buf, len);
    #endif
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

    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_DO_SET_SERVO, 
        0,
        servo,                    
        pwm,
        0,              
        0,          
        0, 0, 0  
    );

    /*
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
    */
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    std::cout << "[MAVLink] Sent servo command: servo=" << (int)servo << ", pwm=" << pwm << "\n";
}

void clearMission() {
    static std::vector<MissionItem> nullMission;
    clearAutopilotMission();
}

/*
TO_DO: THis dosn't work very well
*/
bool setFlightMode(int mode) {
    // 1) pick the right MAV_MODE_* define for param1
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = mode; 

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
    std::cout << "[MAVLink] Sent flight mode command: " << static_cast<int>(mode) << "\n";
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

    if (missionPlan.empty()) { 
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
    }
       
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

/*
void startMissionUpload(std::vector<MissionItem>& missionPlan, uint8_t current_sequence) {
    uint8_t current_seq = current_sequence;

    
    if (current_seq > 0 && current_seq < missionPlan.size()) {
        missionPlan.erase(missionPlan.begin(), missionPlan.begin() + current_seq);
        for (size_t i = 0; i < missionPlan.size(); ++i) {
            missionPlan[i].msg.seq = i;
            missionPlan[i].msg.current = 0;
        }
        
    }
    else if (current_seq >= missionPlan.size()) {
        std::cout << "[MAVLink] All mission items already uploaded.\n";
        return;
    }


    uint16_t totalSize = missionPlan.size();
    

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
        MissionExecState state = shm_ptr->missionUploadState;
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
            shm_ptr->missionUploadState = (nextIndex >= missionPlan.size()) ? COMPLETED : RUNNING;
            pthread_mutex_unlock(&shm_ptr->mutex);
        }

        if (state == COMPLETED) {
            pthread_mutex_lock(&shm_ptr->mutex);
            shm_ptr->missionUploadState = IDLE;
            shm_ptr->missionCount = missionPlan.size();
            shm_ptr->current_seq = 0;
            pthread_mutex_unlock(&shm_ptr->mutex);

            std::cout << "[MAVLink] Mission upload completed successfully.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

*/
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
    std::cout << "[MAVLink] Sent mission stop\n";
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



struct SimParam {
    const char* name;
    float value;
};

SimParam dummyParams[] = {
    { "SYSID_THISMAV", 1.0f },
    { "SIM_SPEED",     2.5f },
    { "TURN_RATE",     45.0f }
};
const int numParams = sizeof(dummyParams) / sizeof(SimParam);

void respondToParamRequest() {
    
    for (int i = 0; i < numParams; ++i) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_message_t msg;
        mavlink_msg_param_value_pack(
            SYSID,
            COMPID,
            &msg,
            dummyParams[i].name,
            dummyParams[i].value,
            MAV_PARAM_TYPE_REAL32,
            numParams,
            i
        );
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(cmd_fd, buf, len);
    }
}


// Helper function to remove completed mission items from the plan
void removeCompletedMissionItems(std::vector<MissionItem>& missionPlan, uint8_t completed_items) {
    if (completed_items == 0) {
        return; // Nothing to remove
    }
    
    if (completed_items >= missionPlan.size()) {
        std::cout << "[MAVLink] All mission items completed - clearing mission plan\n";
        missionPlan.clear();
        return;
    }
    
    // Remove completed items from the front
    missionPlan.erase(missionPlan.begin(), missionPlan.begin() + completed_items);
    std::cout << "[MAVLink] Removed " << static_cast<int>(completed_items) 
              << " completed mission items\n";
    
    // Renumber remaining items sequentially
    for (size_t i = 0; i < missionPlan.size(); ++i) {
        missionPlan[i].msg.seq = i;
        missionPlan[i].msg.current = 0;
    }
}

// Function to clear mission on autopilot
bool clearAutopilotMission() {
    std::cout << "[MAVLink] Clearing autopilot mission\n";
    
    // Send MISSION_CLEAR_ALL
    mavlink_message_t clear_msg;
    uint8_t clear_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_clear_all_pack(
        SYSID, COMPID, &clear_msg,
        TARGET_SYS, TARGET_COMP,
        MAV_MISSION_TYPE_MISSION
    );
    Send(cmd_fd, clear_buf, mavlink_msg_to_send_buffer(clear_buf, &clear_msg));
    std::cout << "[MAVLink] Sent MISSION_CLEAR_ALL\n";
    
    // Send MISSION_COUNT = 0
    mavlink_message_t count_msg;
    uint8_t count_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_count_pack(
        SYSID, COMPID, &count_msg,
        TARGET_SYS, TARGET_COMP,
        0,
        MAV_MISSION_TYPE_MISSION,
        0
    );
    Send(cmd_fd, count_buf, mavlink_msg_to_send_buffer(count_buf, &count_msg));
    std::cout << "[MAVLink] Sent MISSION_COUNT = 0\n";
    
    // Wait for acknowledgment
    while (true) {
        pthread_mutex_lock(&shm_ptr->mutex);
        MissionExecState state = shm_ptr->missionUploadState;
        pthread_mutex_unlock(&shm_ptr->mutex);
        
        if (state == COMPLETED) {
            pthread_mutex_lock(&shm_ptr->mutex);
            shm_ptr->missionUploadState = IDLE;
            shm_ptr->missionCount = 0;
            shm_ptr->current_seq = 0;
            setMissionCurrent(0);
            pthread_mutex_unlock(&shm_ptr->mutex);
            
            std::cout << "[MAVLink] Mission cleared successfully\n";
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// Function to upload mission items to autopilot
bool uploadMissionItems(const std::vector<MissionItem>& missionPlan, size_t start_index = 0) {
    if (start_index >= missionPlan.size()) {
        std::cout << "[MAVLink] No mission items to upload\n";
        return true;
    }
    
    size_t current_index = start_index;
    
    while (true) {
        pthread_mutex_lock(&shm_ptr->mutex);
        MissionExecState state = shm_ptr->missionUploadState;
        pthread_mutex_unlock(&shm_ptr->mutex);
        
        if (state == NEXT && current_index < missionPlan.size()) {
            const auto& item = missionPlan[current_index].msg;
            
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_mission_item_int_pack(
                SYSID, COMPID, &msg,
                TARGET_SYS, TARGET_COMP,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1, item.param2, item.param3, item.param4,
                item.x, item.y, item.z,
                item.mission_type
            );
            
            Send(cmd_fd, buf, mavlink_msg_to_send_buffer(buf, &msg));
            std::cout << "[MAVLink] Uploaded mission item seq=" << item.seq 
                      << " (index=" << current_index << ")\n";
            
            current_index++;
            
            pthread_mutex_lock(&shm_ptr->mutex);
            shm_ptr->missionUploadState = (current_index >= missionPlan.size()) ? COMPLETED : RUNNING;
            pthread_mutex_unlock(&shm_ptr->mutex);
        }
        
        if (state == COMPLETED) {
            std::cout << "[MAVLink] Mission upload completed successfully\n";
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// Full mission upload - replaces entire mission and removes completed items
bool startFullMissionUpload(std::vector<MissionItem>& missionPlan, uint8_t completed_items) {
    std::cout << "[MAVLink] Starting full mission upload\n";
    
    // Handle empty mission
    if (missionPlan.empty()) {
        return clearAutopilotMission();
    }
    
    // Remove completed mission items
    removeCompletedMissionItems(missionPlan, completed_items);
    
    // Check if all items were completed
    if (missionPlan.empty()) {
        return clearAutopilotMission();
    }
    
    // Ensure items are numbered sequentially
    for (size_t i = 0; i < missionPlan.size(); ++i) {
        missionPlan[i].msg.seq = i;
        missionPlan[i].msg.current = 0;
    }
    
    // Clear existing mission
    mavlink_message_t clear_msg;
    uint8_t clear_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_clear_all_pack(
        SYSID, COMPID, &clear_msg,
        TARGET_SYS, TARGET_COMP,
        MAV_MISSION_TYPE_MISSION
    );
    Send(cmd_fd, clear_buf, mavlink_msg_to_send_buffer(clear_buf, &clear_msg));
    std::cout << "[MAVLink] Sent MISSION_CLEAR_ALL\n";
    
    // Send mission count
    mavlink_message_t count_msg;
    uint8_t count_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_count_pack(
        SYSID, COMPID, &count_msg,
        TARGET_SYS, TARGET_COMP,
        missionPlan.size(),
        MAV_MISSION_TYPE_MISSION,
        0
    );
    Send(cmd_fd, count_buf, mavlink_msg_to_send_buffer(count_buf, &count_msg));
    std::cout << "[MAVLink] Sent MISSION_COUNT: " << missionPlan.size() << " items\n";
    
    // Upload mission items
    bool success = uploadMissionItems(missionPlan);
    
    if (success) {
        pthread_mutex_lock(&shm_ptr->mutex);
        shm_ptr->missionUploadState = IDLE;
        shm_ptr->missionCount = missionPlan.size();
        shm_ptr->current_seq = 0;
        setMissionCurrent(0);
        pthread_mutex_unlock(&shm_ptr->mutex);
    }
    
    return success;
}

// Partial mission upload - appends new items to existing mission
bool startPartialMissionUpload(std::vector<MissionItem>& missionPlan, int autopilot_size) {
    std::cout << "[MAVLink] Starting add mission items (append mode)\n";
    
    
    // Calculate how many new items we're adding and new total count
    size_t items_to_add = missionPlan.size() - autopilot_size;
    size_t new_total_count = autopilot_size + items_to_add;
    
    // Renumber the new mission items to start from current sequence
    for (size_t i = autopilot_size; i < missionPlan.size(); ++i) {
        missionPlan[i].msg.seq = i;
        missionPlan[i].msg.current = 0;
    }
    
    std::cout << "[MAVLink] Items will be numbered from " << static_cast<int>(autopilot_size)
              << " to " << (new_total_count - 1) << ", new total: " << new_total_count << "\n";
    
    // Send partial write command to add items from current_seq onward
    mavlink_message_t partial_msg;
    uint8_t partial_buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_mission_write_partial_list_pack(
        SYSID, COMPID, &partial_msg,
        TARGET_SYS, TARGET_COMP,
        autopilot_size,           // Start index for new items
        new_total_count - 1,             // End index (inclusive)
        MAV_MISSION_TYPE_MISSION
    );
    Send(cmd_fd, partial_buf, mavlink_msg_to_send_buffer(partial_buf, &partial_msg));
    std::cout << "[MAVLink] Sent MISSION_WRITE_PARTIAL_LIST: " << static_cast<int>(autopilot_size)
              << " to " << (new_total_count - 1) << "\n";
    
    // Upload the new mission items starting from index 0 in our missionPlan vector
    // but they will be numbered starting from current_mission_count
    bool success = uploadMissionItems(missionPlan, autopilot_size);
    
    if (success) {
        pthread_mutex_lock(&shm_ptr->mutex);
        shm_ptr->missionUploadState = IDLE;
        shm_ptr->missionCount = new_total_count;  // Update total count
        pthread_mutex_unlock(&shm_ptr->mutex);
        
        std::cout << "[MAVLink] Successfully added " << items_to_add 
                  << " mission items starting from sequence " << static_cast<int>(autopilot_size)
                  << ". Total mission count now: " << new_total_count << "\n";
    }
    
    return success;
}