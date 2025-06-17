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
    shm_ptr->missionUploadState = IDLE;
    shm_ptr->missionCountAutopilot = 0;
    shm_ptr->missionCount = 0;
    shm_ptr->current_seq  = 0;
    shm_ptr->armed        = false;
    shm_ptr->mode         = 0;
    shm_ptr->mission_state = MISSION_UNKNOWN;
    shm_ptr->requestParams = false;
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
#if SIM_MODE
    if (uart_fd < 0) {
        std::cerr << "[TCP_MODE] init_uart: uart_fd not valid — TCP connection was not accepted in parent\n";
        return -1;
    }

    std::cout << "[TCP_MODE] init_uart: using inherited TCP socket fd=" << uart_fd << "\n";
    return 0;
#else
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
#endif
}


void readerLoop() {
    auto next_hb = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[128];
    
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
                    #if DEBUG_COMMAND_MESSAGES
                                std::cout << "[COMMAND_ACK] command=" << (int)ack.command 
                                        << " result=" << (int)ack.result 
                                        << " progress=" << (int)ack.progress 
                                        << " result_param2=" << ack.result_param2 
                                        << " target_sys=" << (int)ack.target_system 
                                        << " target_comp=" << (int)ack.target_component << "\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                                mavlink_global_position_int_t gp;
                                mavlink_msg_global_position_int_decode(&msg, &gp);
                                shm_ptr->lat = gp.lat / 1e7;
                                shm_ptr->lon = gp.lon / 1e7;
                                shm_ptr->alt = gp.alt / 1000.0;
                          
                    #if DEBUG_POSITION_MESSAGES
                                std::cout << "[GLOBAL_POSITION] lat=" << shm_ptr->lat 
                                        << " lon=" << shm_ptr->lon 
                                        << " alt=" << shm_ptr->alt 
                                        << " rel_alt=" << gp.relative_alt / 1000.0
                                        << " vx=" << gp.vx << " vy=" << gp.vy << " vz=" << gp.vz
                                        << " hdg=" << gp.hdg / 100.0 << "°\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_ATTITUDE: {
                                mavlink_attitude_t att;
                                mavlink_msg_attitude_decode(&msg, &att);
                                shm_ptr->yaw_deg = att.yaw * 180.0f / M_PI;
                              
                    #if DEBUG_ATTITUDE_MESSAGES
                                std::cout << "[ATTITUDE] roll=" << att.roll * 180.0f / M_PI << "° "
                                        << "pitch=" << att.pitch * 180.0f / M_PI << "° "
                                        << "yaw=" << shm_ptr->yaw_deg << "° "
                                        << "rollspeed=" << att.rollspeed * 180.0f / M_PI << "°/s "
                                        << "pitchspeed=" << att.pitchspeed * 180.0f / M_PI << "°/s "
                                        << "yawspeed=" << att.yawspeed * 180.0f / M_PI << "°/s\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_HEARTBEAT: {
                                mavlink_heartbeat_t hb;
                                mavlink_msg_heartbeat_decode(&msg, &hb);
                                if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
                                    shm_ptr->armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                                    shm_ptr->mode = hb.custom_mode;
                                }


                    #if DEBUG_SYSTEM_MESSAGES

                                std::cout << "[HEARTBEAT] type=" << (int)hb.type 
                                        << " autopilot=" << (int)hb.autopilot
                                        << " base_mode=" << (int)hb.base_mode
                                        << " custom_mode=" << hb.custom_mode
                                        << " system_status=" << (int)hb.system_status
                                        << " mavlink_version=" << (int)hb.mavlink_version << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_SYS_STATUS: {
                                mavlink_sys_status_t sys;
                                mavlink_msg_sys_status_decode(&msg, &sys);
                    #if DEBUG_SYSTEM_MESSAGES
                                std::cout << "[SYS_STATUS] voltage=" << sys.voltage_battery / 1000.0f << "V "
                                        << "current=" << sys.current_battery / 100.0f << "A "
                                        << "battery=" << (int)sys.battery_remaining << "% "
                                        << "drop_rate=" << sys.drop_rate_comm << "% "
                                        << "errors_comm=" << sys.errors_comm
                                        << " load=" << sys.load / 10.0f << "%\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_SYSTEM_TIME: {
                                mavlink_system_time_t time;
                                mavlink_msg_system_time_decode(&msg, &time);
                    #if DEBUG_SYSTEM_MESSAGES
                                std::cout << "[SYSTEM_TIME] time_unix=" << time.time_unix_usec / 1000000.0
                                        << " time_boot=" << time.time_boot_ms / 1000.0 << "s\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_GPS_RAW_INT: {
                                mavlink_gps_raw_int_t gps;
                                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    #if DEBUG_POSITION_MESSAGES
                                std::cout << "[GPS_RAW] fix_type=" << (int)gps.fix_type
                                        << " lat=" << gps.lat / 1e7
                                        << " lon=" << gps.lon / 1e7
                                        << " alt=" << gps.alt / 1000.0 << "m"
                                        << " eph=" << gps.eph / 100.0 << "m"
                                        << " epv=" << gps.epv / 100.0 << "m"
                                        << " vel=" << gps.vel / 100.0 << "m/s"
                                        << " cog=" << gps.cog / 100.0 << "°"
                                        << " satellites=" << (int)gps.satellites_visible << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_RAW_IMU: {
                                mavlink_raw_imu_t imu;
                                mavlink_msg_raw_imu_decode(&msg, &imu);
                    #if DEBUG_SENSOR_MESSAGES
                                std::cout << "[RAW_IMU] acc_x=" << imu.xacc << " acc_y=" << imu.yacc << " acc_z=" << imu.zacc
                                        << " gyro_x=" << imu.xgyro << " gyro_y=" << imu.ygyro << " gyro_z=" << imu.zgyro
                                        << " mag_x=" << imu.xmag << " mag_y=" << imu.ymag << " mag_z=" << imu.zmag
                                        << " time=" << imu.time_usec << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_SCALED_PRESSURE: {
                                mavlink_scaled_pressure_t pressure;
                                mavlink_msg_scaled_pressure_decode(&msg, &pressure);
                    #if DEBUG_SENSOR_MESSAGES
                                std::cout << "[SCALED_PRESSURE] press_abs=" << pressure.press_abs << "hPa "
                                        << "press_diff=" << pressure.press_diff << "hPa "
                                        << "temperature=" << pressure.temperature / 100.0f << "°C "
                                        << "time=" << pressure.time_boot_ms << "ms\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                                mavlink_local_position_ned_t local;
                                mavlink_msg_local_position_ned_decode(&msg, &local);
                    #if DEBUG_POSITION_MESSAGES
                                std::cout << "[LOCAL_POSITION_NED] x=" << local.x << "m y=" << local.y << "m z=" << local.z << "m "
                                        << "vx=" << local.vx << "m/s vy=" << local.vy << "m/s vz=" << local.vz << "m/s "
                                        << "time=" << local.time_boot_ms << "ms\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_MISSION_COUNT: {
                                mavlink_mission_count_t mc;
                                mavlink_msg_mission_count_decode(&msg, &mc);
                                shm_ptr->missionUploadState = RUNNING;
                    #if DEBUG_MISSION_MESSAGES
                                std::cout << "[MISSION_COUNT] count=" << mc.count 
                                        << " target_sys=" << (int)mc.target_system
                                        << " target_comp=" << (int)mc.target_component
                                        << " mission_type=" << (int)mc.mission_type << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
                                mavlink_mission_request_int_t req;
                                mavlink_msg_mission_request_int_decode(&msg, &req);
                                shm_ptr->missionUploadState = NEXT;
                    #if DEBUG_MISSION_MESSAGES
                                std::cout << "[MISSION_REQUEST_INT] seq=" << req.seq
                                        << " target_sys=" << (int)req.target_system
                                        << " target_comp=" << (int)req.target_component
                                        << " mission_type=" << (int)req.mission_type << "\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_MISSION_REQUEST: {
                                mavlink_mission_request_t req;
                                mavlink_msg_mission_request_decode(&msg, &req);
                                shm_ptr->missionUploadState = NEXT;
                    #if DEBUG_MISSION_MESSAGES
                                std::cout << "[MISSION_REQUEST] seq=" << req.seq
                                        << " target_sys=" << (int)req.target_system
                                        << " target_comp=" << (int)req.target_component
                                        << " mission_type=" << (int)req.mission_type << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_MISSION_ACK: {
                                mavlink_mission_ack_t ack;
                                mavlink_msg_mission_ack_decode(&msg, &ack);
                                
                    #if DEBUG_MISSION_MESSAGES
                                std::cout << "[MISSION_ACK] type=" << (int)ack.type 
                                        << " mission_type=" << (int)ack.mission_type
                                        << " target_sys=" << (int)ack.target_system
                                        << " target_comp=" << (int)ack.target_component << "\n";
                    #endif
                                if ((int)ack.type == 0 && (int)ack.mission_type == 0) {
                                    std::cout << "[DEBUG] Mission upload completed\n";
                                    shm_ptr->missionUploadState = COMPLETED;
                                }
                                break;
                            }

                            case MAVLINK_MSG_ID_MISSION_CURRENT: {
                                mavlink_mission_current_t mc;
                                mavlink_msg_mission_current_decode(&msg, &mc);
                                shm_ptr->missionCountAutopilot = mc.total+1;
                                shm_ptr->current_seq = mc.seq;
                                shm_ptr->mission_state = static_cast<MissionState>(mc.mission_state);
                    #if DEBUG_MISSION_MESSAGES
                                
                                std::cout << "[MISSION_CURRENT] seq=" << mc.seq 
                                        << " total=" << mc.total
                                        << " mission_state=" << (int)mc.mission_state
                                        << " mission_mode=" << (int)mc.mission_mode << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
                                mavlink_mission_item_reached_t mir;
                                mavlink_msg_mission_item_reached_decode(&msg, &mir);
                    #if DEBUG_MISSION_MESSAGES
                                std::cout << "[MISSION_ITEM_REACHED] seq=" << mir.seq << " (next=" << shm_ptr->current_seq << ")\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
                                mavlink_rc_channels_raw_t rc;
                                mavlink_msg_rc_channels_raw_decode(&msg, &rc);
                    #if DEBUG_RC_SERVO_MESSAGES
                                std::cout << "[RC_CHANNELS_RAW] ch1=" << rc.chan1_raw << " ch2=" << rc.chan2_raw 
                                        << " ch3=" << rc.chan3_raw << " ch4=" << rc.chan4_raw
                                        << " ch5=" << rc.chan5_raw << " ch6=" << rc.chan6_raw
                                        << " ch7=" << rc.chan7_raw << " ch8=" << rc.chan8_raw
                                        << " rssi=" << (int)rc.rssi << " port=" << (int)rc.port << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
                                mavlink_servo_output_raw_t servo;
                                mavlink_msg_servo_output_raw_decode(&msg, &servo);
                    #if DEBUG_RC_SERVO_MESSAGES
                                std::cout << "[SERVO_OUTPUT_RAW] servo6=" << servo.servo6_raw
                                        << " servo7=" << servo.servo7_raw << " servo8=" << servo.servo8_raw
                                        << " port=" << (int)servo.port << " time=" << servo.time_usec << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_VFR_HUD: {
                                mavlink_vfr_hud_t vfr;
                                mavlink_msg_vfr_hud_decode(&msg, &vfr);
                    #if DEBUG_STATUS_MESSAGES
                                std::cout << "[VFR_HUD] airspeed=" << vfr.airspeed << "m/s "
                                        << "groundspeed=" << vfr.groundspeed << "m/s "
                                        << "heading=" << vfr.heading << "° "
                                        << "throttle=" << vfr.throttle << "% "
                                        << "alt=" << vfr.alt << "m "
                                        << "climb=" << vfr.climb << "m/s\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_SCALED_IMU3: {
                                mavlink_scaled_imu3_t imu3;
                                mavlink_msg_scaled_imu3_decode(&msg, &imu3);
                    #if DEBUG_SENSOR_MESSAGES
                                std::cout << "[SCALED_IMU3] acc_x=" << imu3.xacc << " acc_y=" << imu3.yacc << " acc_z=" << imu3.zacc
                                        << " gyro_x=" << imu3.xgyro << " gyro_y=" << imu3.ygyro << " gyro_z=" << imu3.zgyro
                                        << " mag_x=" << imu3.xmag << " mag_y=" << imu3.ymag << " mag_z=" << imu3.zmag
                                        << " temp=" << imu3.temperature << "°C time=" << imu3.time_boot_ms << "ms\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_POWER_STATUS: {
                                mavlink_power_status_t power;
                                mavlink_msg_power_status_decode(&msg, &power);
                    #if DEBUG_SYSTEM_MESSAGES
                                std::cout << "[POWER_STATUS] Vcc=" << power.Vcc << "mV "
                                        << "Vservo=" << power.Vservo << "mV "
                                        << "flags=" << power.flags << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_SCALED_PRESSURE3: {
                                mavlink_scaled_pressure3_t press3;
                                mavlink_msg_scaled_pressure3_decode(&msg, &press3);
                    #if DEBUG_SENSOR_MESSAGES
                                std::cout << "[SCALED_PRESSURE3] press_abs=" << press3.press_abs << "hPa "
                                        << "press_diff=" << press3.press_diff << "hPa "
                                        << "temperature=" << press3.temperature / 100.0f << "°C "
                                        << "time=" << press3.time_boot_ms << "ms\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
                                mavlink_ekf_status_report_t ekf;
                                mavlink_msg_ekf_status_report_decode(&msg, &ekf);
                    #if DEBUG_STATUS_MESSAGES
                                std::cout << "[EKF_STATUS] flags=" << ekf.flags
                                        << " velocity_variance=" << ekf.velocity_variance
                                        << " pos_horiz_variance=" << ekf.pos_horiz_variance
                                        << " pos_vert_variance=" << ekf.pos_vert_variance
                                        << " compass_variance=" << ekf.compass_variance
                                        << " terrain_alt_variance=" << ekf.terrain_alt_variance << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_VIBRATION: {
                                mavlink_vibration_t vib;
                                mavlink_msg_vibration_decode(&msg, &vib);
                    #if DEBUG_SENSOR_MESSAGES
                                std::cout << "[VIBRATION] vibe_x=" << vib.vibration_x 
                                        << " vibe_y=" << vib.vibration_y 
                                        << " vibe_z=" << vib.vibration_z
                                        << " clipping_0=" << vib.clipping_0
                                        << " clipping_1=" << vib.clipping_1
                                        << " clipping_2=" << vib.clipping_2
                                        << " time=" << vib.time_usec << "\n";
                    #endif
                                break;
                            }

                            case MAVLINK_MSG_ID_STATUSTEXT: {
                                mavlink_statustext_t txt;
                                mavlink_msg_statustext_decode(&msg, &txt);
                    #if DEBUG_PREARM_MESSAGES
                                std::cout << "[STATUSTEXT] severity=" << (int)txt.severity 
                                        << " text=\"" << txt.text << "\"\n";
                    #endif
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

                    #if DEBUG_COMMAND_MESSAGES
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
                    #endif
                                break;
                            }
                        
                            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
                                mavlink_request_data_stream_t rds;
                                mavlink_msg_request_data_stream_decode(&msg, &rds);
                    #if DEBUG_SYSTEM_MESSAGES
                                std::cout << "[REQUEST_DATA_STREAM] stream_id=" << (int)rds.req_stream_id
                                        << " rate=" << rds.req_message_rate << "Hz"
                                        << " start_stop=" << (int)rds.start_stop
                                        << " target_sys=" << (int)rds.target_system
                                        << " target_comp=" << (int)rds.target_component << "\n";
                    #endif
                                break;
                            }
                            
                            case MAV_CMD_DO_SET_SERVO: {
                    #if DEBUG_COMMAND_MESSAGES
                                std::cout << "[DO_SET_SERVO] Command received\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_MCU_STATUS: {
                                mavlink_mcu_status_t mcu;
                                mavlink_msg_mcu_status_decode(&msg, &mcu);
                    #if DEBUG_SYSTEM_MESSAGES
                                std::cout << "[MCU_STATUS] id=" << (int)mcu.id
                                        << " MCU_temperature=" << mcu.MCU_temperature
                                        << " MCU_voltage=" << mcu.MCU_voltage
                                        << " MCU_voltage_min=" << mcu.MCU_voltage_min
                                        << " MCU_voltage_max=" << mcu.MCU_voltage_max << "\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_PARAM_VALUE: {
                                mavlink_param_value_t param;
                                mavlink_msg_param_value_decode(&msg, &param);
                    #if DEBUG_PARAMETER_MESSAGES
                                std::cout << "[PARAM_VALUE] id=\"" << param.param_id << "\""
                                        << " value=" << param.param_value
                                        << " type=" << (int)param.param_type
                                        << " count=" << param.param_count
                                        << " index=" << param.param_index << "\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
                                mavlink_position_target_global_int_t target;
                                mavlink_msg_position_target_global_int_decode(&msg, &target);
                    #if DEBUG_NAVIGATION_MESSAGES
                                std::cout << "[POSITION_TARGET_GLOBAL_INT] lat=" << target.lat_int / 1e7
                                        << " lon=" << target.lon_int / 1e7
                                        << " alt=" << target.alt
                                        << " vx=" << target.vx << " vy=" << target.vy << " vz=" << target.vz
                                        << " afx=" << target.afx << " afy=" << target.afy << " afz=" << target.afz
                                        << " yaw=" << target.yaw * 180.0f / M_PI << "°"
                                        << " yaw_rate=" << target.yaw_rate * 180.0f / M_PI << "°/s"
                                        << " type_mask=" << target.type_mask
                                        << " coordinate_frame=" << (int)target.coordinate_frame << "\n";
                    #endif
                                break;
                            }
                            
                            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
                                mavlink_nav_controller_output_t nav;
                                mavlink_msg_nav_controller_output_decode(&msg, &nav);
                    #if DEBUG_NAVIGATION_MESSAGES
                                std::cout << "[NAV_CONTROLLER_OUTPUT] nav_roll=" << nav.nav_roll << "°"
                                        << " nav_pitch=" << nav.nav_pitch << "°"
                                        << " nav_bearing=" << nav.nav_bearing << "°"
                                        << " target_bearing=" << nav.target_bearing << "°"
                                        << " wp_dist=" << nav.wp_dist << "m"
                                        << " alt_error=" << nav.alt_error << "m"
                                        << " aspd_error=" << nav.aspd_error << "m/s"
                                        << " xtrack_error=" << nav.xtrack_error << "m\n";
                    #endif
                                break;
                            }
                        case MAVLINK_MSG_ID_RC_CHANNELS: {
                            mavlink_rc_channels_t rc;
                            mavlink_msg_rc_channels_decode(&msg, &rc);
                        #if DEBUG_RC_MESSAGES
                            std::cout << "[RC_CHANNELS] chancount=" << rc.chancount
                                    << " rssi=" << (int)rc.rssi
                                    << " time_boot_ms=" << rc.time_boot_ms << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED: {
                            mavlink_rc_channels_scaled_t rc_scaled;
                            mavlink_msg_rc_channels_scaled_decode(&msg, &rc_scaled);
                        #if DEBUG_RC_MESSAGES
                            std::cout << "[RC_CHANNELS_SCALED] ch1=" << rc_scaled.chan1_scaled
                                    << " ch2=" << rc_scaled.chan2_scaled
                                    << " ch3=" << rc_scaled.chan3_scaled
                                    << " ch4=" << rc_scaled.chan4_scaled
                                    << " ch5=" << rc_scaled.chan5_scaled
                                    << " ch6=" << rc_scaled.chan6_scaled
                                    << " ch7=" << rc_scaled.chan7_scaled
                                    << " ch8=" << rc_scaled.chan8_scaled
                                    << " port=" << (int)rc_scaled.port
                                    << " time_boot_ms=" << rc_scaled.time_boot_ms << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_MEMINFO: {
                            mavlink_meminfo_t mem;
                            mavlink_msg_meminfo_decode(&msg, &mem);
                        #if DEBUG_MISC_MESSAGES
                            std::cout << "[MEMINFO] brkval=" << mem.brkval
                                    << " freemem=" << mem.freemem << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_SCALED_IMU2: {
                            mavlink_scaled_imu2_t imu2;
                            mavlink_msg_scaled_imu2_decode(&msg, &imu2);
                        #if DEBUG_SENSOR_MESSAGES
                            std::cout << "[SCALED_IMU2] acc_x=" << imu2.xacc
                                    << " acc_y=" << imu2.yacc
                                    << " acc_z=" << imu2.zacc
                                    << " gyro_x=" << imu2.xgyro
                                    << " gyro_y=" << imu2.ygyro
                                    << " gyro_z=" << imu2.zgyro
                                    << " mag_x=" << imu2.xmag
                                    << " mag_y=" << imu2.ymag
                                    << " mag_z=" << imu2.zmag
                                    << " time=" << imu2.time_boot_ms << "ms\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_SCALED_PRESSURE2: {
                            mavlink_scaled_pressure2_t press2;
                            mavlink_msg_scaled_pressure2_decode(&msg, &press2);
                        #if DEBUG_SENSOR_MESSAGES
                            std::cout << "[SCALED_PRESSURE2] press_abs=" << press2.press_abs
                                    << "hPa press_diff=" << press2.press_diff
                                    << "hPa temperature=" << press2.temperature / 100.0f
                                    << "°C time=" << press2.time_boot_ms << "ms\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_GPS2_RAW: {
                            mavlink_gps2_raw_t gps2;
                            mavlink_msg_gps2_raw_decode(&msg, &gps2);
                        #if DEBUG_POSITION_MESSAGES
                            std::cout << "[GPS2_RAW] fix_type=" << (int)gps2.fix_type
                                    << " lat=" << gps2.lat / 1e7
                                    << " lon=" << gps2.lon / 1e7
                                    << " alt=" << gps2.alt / 1000.0 << "m"
                                    << " eph=" << gps2.eph / 100.0 << "m"
                                    << " epv=" << gps2.epv / 100.0 << "m"
                                    << " vel=" << gps2.vel / 100.0 << "m/s"
                                    << " cog=" << gps2.cog / 100.0 << "°"
                                    << " satellites=" << (int)gps2.satellites_visible << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_AHRS: {
                            mavlink_ahrs_t ahrs;
                            mavlink_msg_ahrs_decode(&msg, &ahrs);
                        #if DEBUG_STATUS_MESSAGES
                            std::cout << "[AHRS] omegaIx=" << ahrs.omegaIx
                                    << " omegaIy=" << ahrs.omegaIy
                                    << " omegaIz=" << ahrs.omegaIz
                                    << " accel_weight=" << ahrs.accel_weight
                                    << " renorm_val=" << ahrs.renorm_val << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_AHRS2: {
                            mavlink_ahrs2_t ahrs2;
                            mavlink_msg_ahrs2_decode(&msg, &ahrs2);
                        #if DEBUG_STATUS_MESSAGES
                            std::cout << "[AHRS2] roll=" << ahrs2.roll
                                    << " pitch=" << ahrs2.pitch
                                    << " yaw=" << ahrs2.yaw
                                    << " alt=" << ahrs2.altitude
                                    << " lat=" << ahrs2.lat / 1e7
                                    << " lon=" << ahrs2.lng / 1e7 << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_TIMESYNC: {
                            mavlink_timesync_t ts;
                            mavlink_msg_timesync_decode(&msg, &ts);
                        #if DEBUG_SYSTEM_MESSAGES
                            std::cout << "[TIMESYNC] tc1=" << ts.tc1
                                    << " ts1=" << ts.ts1 << "\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: {
                            mavlink_gps_global_origin_t origin;
                            mavlink_msg_gps_global_origin_decode(&msg, &origin);
                        #if DEBUG_POSITION_MESSAGES
                            std::cout << "[GPS_GLOBAL_ORIGIN] lat=" << origin.latitude / 1e7
                                    << " lon=" << origin.longitude / 1e7
                                    << " alt=" << origin.altitude / 1000.0 << "m\n";
                        #endif
                            break;
                        }

                        case MAVLINK_MSG_ID_HOME_POSITION: {
                            mavlink_home_position_t home;
                            mavlink_msg_home_position_decode(&msg, &home);
                        #if DEBUG_MISC_MESSAGES
                            std::cout << "[HOME_POSITION] lat=" << home.latitude / 1e7
                                    << " lon=" << home.longitude / 1e7
                                    << " alt=" << home.altitude / 1000.0
                                    << " approach_x=" << home.approach_x
                                    << " approach_y=" << home.approach_y
                                    << " approach_z=" << home.approach_z << "\n";
                        #endif
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                            shm_ptr->requestParams = true;
                            std::cout << "[PARAM_REQUEST_LIST]" << std::endl;
                            break;
                        }
                        case MAVLINK_MSG_ID_SIMSTATE: {
                            mavlink_simstate_t simstate;
                            mavlink_msg_simstate_decode(&msg, &simstate);
                        #if DEBUG_SIM_MESSAGES
                            std::cout << "[SIMSTATE] roll=" << simstate.roll
                                    << " pitch=" << simstate.pitch
                                    << " yaw=" << simstate.yaw
                                    << " lat=" << simstate.lat / 1e7
                                    << " xacc=" << simstate.xacc
                                    << " yacc=" << simstate.yacc
                                    << " zacc=" << simstate.zacc
                                    << "\n";
                        #endif
                            break;
                        }
                        case  MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {
                            mavlink_file_transfer_protocol_t ftp;
                            mavlink_msg_file_transfer_protocol_decode(&msg, &ftp);
                            std::cout << "[FILE_TRANSFER_PROTOCOL] target_network=" << (int)ftp.target_network
                                    << " target_system=" << (int)ftp.target_system
                                    << " target_component=" << (int)ftp.target_component
                                    << " payload_length=" << sizeof(ftp.payload) << "\n";
                            break;

                        }
                               
                            default:
                                
                                std::cout << "[UNKNOWN] msgid=" << msg.msgid
                                        << " compid=" << static_cast<int>(msg.compid)
                                        << " sysid=" << static_cast<int>(msg.sysid) << "\n";
                                
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