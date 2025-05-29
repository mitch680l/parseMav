#include "reader.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <poll.h>
#include <chrono>
int uart_fd = -1;
// Shared-memory structure
SharedTelem* shm_ptr = nullptr;
constexpr size_t RING_BUFFER_SIZE = 1024;
struct RingBuffer {
    uint8_t data[RING_BUFFER_SIZE];
    size_t head = 0;
    size_t tail = 0;

    void push(uint8_t byte) {
        data[head] = byte;
        head = (head + 1) % RING_BUFFER_SIZE;
        if (head == tail)  // overwrite old data if full
            tail = (tail + 1) % RING_BUFFER_SIZE;
    }

    bool pop(uint8_t& byte) {
        if (head == tail) return false;  // empty
        byte = data[tail];
        tail = (tail + 1) % RING_BUFFER_SIZE;
        return true;
    }

    bool available() const {
        return head != tail;
    }
} ring;

bool create_ipc() {
    shm_unlink(ShM_NAME);
    int fd = shm_open(ShM_NAME, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd < 0) {
        std::cerr << "[ERROR] shm_open(create) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    if (ftruncate(fd, sizeof(SharedTelem)) != 0) {
        std::cerr << "[ERROR] ftruncate failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    void* addr = mmap(nullptr, sizeof(SharedTelem), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        std::cerr << "[ERROR] mmap(create) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    shm_ptr = reinterpret_cast<SharedTelem*>(addr);

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm_ptr->mutex, &attr);

    // Initialize fields
    shm_ptr->lat          = 0.0;
    shm_ptr->lon          = 0.0;
    shm_ptr->alt          = 0.0;
    shm_ptr->yaw_deg      = 0.0;
    shm_ptr->seq          = 0;
    shm_ptr->missionState = IDLE;
    shm_ptr->missionCount = 0;
    shm_ptr->current_seq  = 0;
    return true;
}

bool open_ipc() {
    int fd = shm_open(ShM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        std::cerr << "[ERROR] shm_open(open) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    void* addr = mmap(nullptr, sizeof(SharedTelem), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        std::cerr << "[ERROR] mmap(open) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    shm_ptr = reinterpret_cast<SharedTelem*>(addr);
    return true;
}

int init_uart() {
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        std::cerr << "[ERROR] Failed to open UART: " << strerror(errno) << "\n";
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(uart_fd, &tty) != 0)  {
        std::cerr << "[ERROR] tcgetattr failed: " << strerror(errno) << "\n";
        close(uart_fd);
        return -1;
    }

    // Set baud rate
    cfsetospeed(&tty, BAUD);
    cfsetispeed(&tty, BAUD);

    // Raw mode
    cfmakeraw(&tty);

    // Set 8N1: 8 data bits, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);    // enable receiver, ignore modem status lines
    tty.c_cflag &= ~(PARENB | PARODD);  // no parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // no hardware flow control

    // Input flags - turn off flow control, disable CR->NL translation, etc.
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR); // don't mangle CR or NL

    // Output flags - raw output
    tty.c_oflag = 0;

    // Local flags - disable canonical mode, echo, signals
    tty.c_lflag = 0;

    // Non-blocking read with timeout
    tty.c_cc[VMIN]  = 0;    // return as soon as data is available
    tty.c_cc[VTIME] = 1;    // timeout = 0.1 sec

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed: " << strerror(errno) << "\n";
        close(uart_fd);
        return -1;
    }

    // Flush any garbage in the buffer
    tcflush(uart_fd, TCIFLUSH);
    return 0;
}


void readerLoop() {
    auto next_hb = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[128];
    uint8_t hb_buf[MAVLINK_MAX_PACKET_LEN];
    size_t hb_len;
    struct pollfd pfd{ .fd = uart_fd, .events = POLLIN };

    while (true) {
        // Wait until next heartbeat or UART input
        auto now = std::chrono::steady_clock::now();
        int timeout = std::chrono::duration_cast<std::chrono::milliseconds>(next_hb - now).count();
        if (timeout < 0) timeout = 0;
        int ret = poll(&pfd, 1, timeout);
        if (ret < 0) {
            std::cerr << "[ERROR] poll: " << strerror(errno) << "\n";
            continue;
        }

        // Send GCS heartbeat every second
        now = std::chrono::steady_clock::now();
        if (now >= next_hb) {
            mavlink_message_t hb_msg;
            mavlink_msg_heartbeat_pack(
                SYSID, COMPID, &hb_msg,
                MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                0, 0, MAV_STATE_ACTIVE
            );
            hb_len = mavlink_msg_to_send_buffer(hb_buf, &hb_msg);
            Send(uart_fd, hb_buf, hb_len);
            next_hb = now + std::chrono::seconds(1);
        }

        // If UART has input, read and enqueue into ring buffer
        if (ret > 0 && (pfd.revents & POLLIN)) {
            int len = read(uart_fd, buf, sizeof(buf));
            if (len > 0) {
                for (int i = 0; i < len; ++i)
                    ring.push(buf[i]);
            } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "[ERROR] read failed: " << strerror(errno) << "\n";
            }
        }

        // MAVLink byte-by-byte parser
        uint8_t byte;
        while (ring.pop(byte)) {

            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                pthread_mutex_lock(&shm_ptr->mutex);
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_COMMAND_ACK: {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&msg, &ack);
                        std::cout << "[DEBUG] Command ACK received, command=" << (int)ack.command << ", result=" << (int)ack.result << "\n";
                        break;
                    }
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                        mavlink_global_position_int_t gp;
                        mavlink_msg_global_position_int_decode(&msg, &gp);
                        shm_ptr->lat = gp.lat / 1e7;
                        shm_ptr->lon = gp.lon / 1e7;
                        shm_ptr->alt = gp.alt / 1000.0;
                        shm_ptr->seq++;
                        break;
                    }
                    
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg, &att);
                        shm_ptr->yaw_deg = att.yaw * 180.0f / M_PI;
                        shm_ptr->seq++;
                        break;
                    }

                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        break;
                    }

                    case MAVLINK_MSG_ID_SYS_STATUS: {
                        mavlink_sys_status_t sys;
                        mavlink_msg_sys_status_decode(&msg, &sys);
                        break;
                    }

                    case MAVLINK_MSG_ID_SYSTEM_TIME: {
                        mavlink_system_time_t time;
                        mavlink_msg_system_time_decode(&msg, &time);
                        break;
                    }

                    case MAVLINK_MSG_ID_GPS_RAW_INT: {
                        mavlink_gps_raw_int_t gps;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps);
                        break;
                    }

                    case MAVLINK_MSG_ID_RAW_IMU: {
                        mavlink_raw_imu_t imu;
                        mavlink_msg_raw_imu_decode(&msg, &imu);
                        break;
                    }

                    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
                        mavlink_scaled_pressure_t pressure;
                        mavlink_msg_scaled_pressure_decode(&msg, &pressure);
                        break;
                    }

                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                        mavlink_local_position_ned_t local;
                        mavlink_msg_local_position_ned_decode(&msg, &local);
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_COUNT: {
                        mavlink_mission_count_t mc;
                        mavlink_msg_mission_count_decode(&msg, &mc);
                        shm_ptr->missionState = RUNNING;
                        std::cout << "[DEBUG] Mission count received: " << mc.count << "\n";
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
                        mavlink_mission_request_int_t req;
                        mavlink_msg_mission_request_int_decode(&msg, &req);
                        std::cout << "[DEBUG] Mission request for seq: " << req.seq << "\n";
                        shm_ptr->missionState = NEXT;
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST: {
                        mavlink_mission_request_int_t req;
                        mavlink_msg_mission_request_int_decode(&msg, &req);
                        std::cout << "[DEBUG] Mission request for seq: " << req.seq << "\n";
                        shm_ptr->missionState = NEXT;
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_ACK: {
                        mavlink_mission_ack_t ack;
                        mavlink_msg_mission_ack_decode(&msg, &ack);
                        std::cout << "[DEBUG] Mission ACK received, type=" << (int)ack.type << " Mission: "<< (int)ack.mission_type << "\n";
                        if ((int)ack.type == 0 && (int)ack.mission_type == 0) 
                        {
                            std::cout << "[DEBUG] Mission upload completed\n";
                            shm_ptr->missionState = COMPLETED;
                        }
                        
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_CURRENT: {
                        mavlink_mission_current_t mc;
                        mavlink_msg_mission_current_decode(&msg, &mc);
                        shm_ptr->seq = mc.seq;
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
                        mavlink_mission_item_reached_t mir;
                        mavlink_msg_mission_item_reached_decode(&msg, &mir);
                        std::cout << "[DEBUG] Mission item reached seq: " << mir.seq << "\n";
                        shm_ptr->current_seq = mir.seq + 1;
                        break;
                    }
                    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
                        mavlink_rc_channels_raw_t rc;
                        mavlink_msg_rc_channels_raw_decode(&msg, &rc);
                        break;
                    }

                    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
                        mavlink_servo_output_raw_t servo;
                        mavlink_msg_servo_output_raw_decode(&msg, &servo);
                        break;
                    }

                    case MAVLINK_MSG_ID_VFR_HUD: {
                        mavlink_vfr_hud_t vfr;
                        mavlink_msg_vfr_hud_decode(&msg, &vfr);
                        break;
                    }

                    case MAVLINK_MSG_ID_SCALED_IMU3: {
                        mavlink_scaled_imu3_t imu3;
                        mavlink_msg_scaled_imu3_decode(&msg, &imu3);
                        break;
                    }

                    case MAVLINK_MSG_ID_POWER_STATUS: {
                        mavlink_power_status_t power;
                        mavlink_msg_power_status_decode(&msg, &power);
                        break;
                    }

                    case MAVLINK_MSG_ID_SCALED_PRESSURE3: {
                        mavlink_scaled_pressure3_t press3;
                        mavlink_msg_scaled_pressure3_decode(&msg, &press3);
                        break;
                    }

                    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
                        mavlink_ekf_status_report_t ekf;
                        mavlink_msg_ekf_status_report_decode(&msg, &ekf);
                        break;
                    }

                    case MAVLINK_MSG_ID_VIBRATION: {
                        mavlink_vibration_t vib;
                        mavlink_msg_vibration_decode(&msg, &vib);
                        break;
                    }

                    case MAVLINK_MSG_ID_STATUSTEXT: {
                        mavlink_statustext_t txt;
                        mavlink_msg_statustext_decode(&msg, &txt);
                        break;
                    }

                    case MAVLINK_MSG_ID_COMMAND_LONG: {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);

                        std::string cmd_name;

                        switch (cmd.command) {
                            case MAV_CMD_COMPONENT_ARM_DISARM:
                                cmd_name = "ARM/DISARM";
                                break;
                            case MAV_CMD_NAV_TAKEOFF:
                                cmd_name = "TAKEOFF";
                                break;
                            case MAV_CMD_NAV_LAND:
                                cmd_name = "LAND";
                                break;
                            case MAV_CMD_MISSION_START:
                                cmd_name = "MISSION_START";
                                break;
                            case MAV_CMD_DO_SET_MODE:
                                cmd_name = "SET_MODE";
                                break;
                            case MAV_CMD_DO_PAUSE_CONTINUE:
                                cmd_name = "PAUSE/CONTINUE";
                                break;
                            case MAV_CMD_DO_CHANGE_SPEED:
                                cmd_name = "CHANGE_SPEED";
                                break;
                            case MAV_CMD_DO_SET_HOME:
                                cmd_name = "SET_HOME";
                                break;
                            default:
                                cmd_name = "UNKNOWN_COMMAND";
                                break;
                        }

                        std::cout << "[COMMAND_LONG] cmd=" << cmd.command
                                << " (" << cmd_name << ")"
                                << " target_sys=" << (int)cmd.target_system
                                << " target_comp=" << (int)cmd.target_component
                                << " param1=" << cmd.param1
                                << " param2=" << cmd.param2
                                << " param3=" << cmd.param3
                                << " param4=" << cmd.param4
                                << " param5=" << cmd.param5
                                << " param6=" << cmd.param6
                                << " param7=" << cmd.param7 << "\n";
                        break;
                    }
                  
                    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
                        break;
                    }
                    case MAV_CMD_DO_SET_SERVO: {
                        break;
                    }
                    case MAVLINK_MSG_ID_MCU_STATUS: {
                        //MCU status
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_VALUE: {
                        break;
                    }
                    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
                        break;
                    }
                    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
                        
                        break;
                    }
                    default:
                        if (msg.msgid == 152 || msg.msgid == 65 || msg.msgid == 116 || msg.msgid == 137 || msg.msgid == 124 || msg.msgid == 34 || msg.msgid == 163 || msg.msgid == 178 || msg.msgid == 111) {
                            // Ignore these messages
                        }
                        else {
                            std::cout << "[DEBUG] Unknown message ID: " << msg.msgid
                                << " Compid: " << static_cast<int>(msg.compid)
                                << " Sysid: " << static_cast<int>(msg.sysid) << "\n";
                        }
                        break;
                }
                pthread_mutex_unlock(&shm_ptr->mutex);
            }
        }
    }
}


void getTelem(double* lat, double* lon, double* alt) {
    pthread_mutex_lock(&shm_ptr->mutex);
    *lat = shm_ptr->lat;
    *lon = shm_ptr->lon;
    *alt = shm_ptr->alt;
    pthread_mutex_unlock(&shm_ptr->mutex);
}

void getBearing(double* bearing) {
    pthread_mutex_lock(&shm_ptr->mutex);
    *bearing = shm_ptr->yaw_deg;
    pthread_mutex_unlock(&shm_ptr->mutex);
}

uint64_t getSeq() {
    pthread_mutex_lock(&shm_ptr->mutex);
    uint64_t s = shm_ptr->seq;
    pthread_mutex_unlock(&shm_ptr->mutex);
    return s;
}

void setCurrent(int current) {
    pthread_mutex_lock(&shm_ptr->mutex);
    shm_ptr->current_seq = current;
    pthread_mutex_unlock(&shm_ptr->mutex);
}