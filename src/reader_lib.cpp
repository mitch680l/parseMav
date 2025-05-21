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
static const char* SHM_NAME = ShM_NAME;
struct SharedTelem {
    pthread_mutex_t mutex;
    double lat, lon, alt;
    double yaw_deg;
    uint64_t seq;
};
static SharedTelem* shm_ptr = nullptr;
static int msg_count = 0;
void requestStreams() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t len;

    mavlink_msg_heartbeat_pack(
        SYSID, COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0,
        MAV_STATE_ACTIVE
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    
    Send(uart_fd, buf, len);
    std::cout << "[DEBUG] Sending Heartbeat\n";

    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_SET_MESSAGE_INTERVAL,  // 511
        0,                             // confirmation
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, // param1: message ID (33)
        50000,                         // param2: interval in µs (50 000 µs → 20 Hz)
        0, 0, 0, 0,                    // param3–6: unused
        0                              // param7: target (0 = autopilot default)
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(uart_fd, buf, len);
    std::cout << "[DEBUG] Sending SET_MESSAGE_INTERVAL for GLOBAL_POSITION_INT\n";
    mavlink_msg_command_long_pack(
        SYSID, COMPID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_SET_MESSAGE_INTERVAL,  // 511
        0,
        MAVLINK_MSG_ID_ATTITUDE,       // param1: message ID (30)
        50000,                         // param2: 20 Hz
        0, 0, 0, 0,
        0
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(uart_fd, buf, len);
    std::cout << "[DEBUG] Sending SET_MESSAGE_INTERVAL for AVLINK_MSG_ID_ATTITUDE\n";

}

bool create_ipc() {
    shm_unlink(SHM_NAME);
    // 1) Create and size
    int fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0666);
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
    void* addr = mmap(nullptr, sizeof(SharedTelem),
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        std::cerr << "[ERROR] mmap(create) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    shm_ptr = reinterpret_cast<SharedTelem*>(addr);

    // 3) Initialize the mutex for process‐shared use
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm_ptr->mutex, &attr);

    // 4) Zero out your struct
    shm_ptr->lat = shm_ptr->lon = shm_ptr->alt = 0.0;
    shm_ptr->yaw_deg = 0.0;
    shm_ptr->seq = 0;
    return true;
}

bool open_ipc() {
    // 1) Open only (no create, no truncate)
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        std::cerr << "[ERROR] shm_open(open) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    // 2) Map exactly same size
    void* addr = mmap(nullptr, sizeof(SharedTelem),
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        std::cerr << "[ERROR] mmap(open) failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    shm_ptr = reinterpret_cast<SharedTelem*>(addr);
    return true;
}

int init_uart() {
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (uart_fd < 0) {
        std::cerr << "[ERROR] Failed to open UART: " << strerror(errno) << "\n";
        return -1;
    }
    std::cout << "[DEBUG] UART opened\n";
    struct termios tty;
    if (tcgetattr(uart_fd, &tty) != 0)  {
        std::cerr << "[ERROR] Failed to open tty: " << strerror(errno) << "\n";
        return -1;
    }
    std::cout << "[DEBUG] tty opened\n";
    cfsetospeed(&tty, BAUD);
    cfsetispeed(&tty, BAUD);
    cfmakeraw(&tty);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;   // Non-blocking read
    tty.c_cc[VTIME] = 1;  // timeout (0.1 seconds)

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed: " << strerror(errno) << "\n";
        close(uart_fd);
        return -1;
    }
    std::cout << "[DEBUG] tty configured\n";
    requestStreams();
    std::cout << "[DEBUG] requestStreams sent\n";
    return 0;
}


void readerLoop() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[255];
    struct pollfd pfd{ .fd = uart_fd, .events = POLLIN };
    
    while (true) {
        int ret = poll(&pfd, 1, 1000);
        if (ret < 0) {
            std::cerr << "[ERROR] poll: " << strerror(errno) << "\n";
            continue;
        }
        if (ret == 0) {
            std::cerr << "[WARN] no data in 1 s\n";
            continue;
        }
        if (!(pfd.revents & POLLIN)) {
            // poll triggered, but no POLLIN event
            std::cerr << "[WARN] poll triggered without POLLIN event\n";
            continue;
        }
        
        int len = read(uart_fd, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available yet; not an error
                continue;
            } else {
                std::cerr << "[ERROR] read failed: " << strerror(errno) << "\n";
                continue;
            }
        } else if (len == 0) {
            // End of file (shouldn't happen with UART)
            std::cerr << "[WARN] read returned EOF\n";
            continue;
        }
        
        for (int i = 0; i < len; i++) {
            
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                pthread_mutex_lock(&shm_ptr->mutex);
                if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t gp;
                    mavlink_msg_global_position_int_decode(&msg, &gp);
                    shm_ptr->lat = gp.lat / 1e7;
                    shm_ptr->lon = gp.lon / 1e7;
                    shm_ptr->alt = gp.alt / 1000.0;
                    shm_ptr->seq++;
                    //std::cout << "[DEBUG] Lat: " << std::fixed << std::setprecision(7) << shm_ptr->lat
                    //     << ", Lon: " << std::fixed << std::setprecision(7) << shm_ptr->lon
                     //    << ", Alt: " << std::fixed << std::setprecision(2) << shm_ptr->alt
                     //    << ", Seq: " << shm_ptr->seq << "\n";
                } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    shm_ptr->yaw_deg = att.yaw * 180.0f / M_PI;
                    shm_ptr->seq++;
                }
                else {
                    //std::cout <<"Uknown" << std::endl;
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

void parseModeAck(const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        if (ack.command == MAV_CMD_DO_SET_MODE) {
            if (ack.result == MAV_RESULT_ACCEPTED)
                std::cout << "[DEBUG] Mode change accepted\n";
            else
                std::cout << "[DEBUG] Mode change failed (" << ack.result << ")\n";
        }
    }
}

void parseThrottleAck(const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        if (ack.command == MAV_CMD_DO_CHANGE_SPEED) {
            if (ack.result == MAV_RESULT_ACCEPTED)
                std::cout << "[DEBUG] Throttle change accepted\n";
            else
                std::cout << "[DEBUG] Throttle change failed (" << ack.result << ")\n";
        }
    }
}


