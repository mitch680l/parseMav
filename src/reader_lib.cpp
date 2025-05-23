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

static SharedTelem* shm_ptr = nullptr;

bool create_ipc() {
    shm_unlink(SHM_NAME);
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
    return true;
}

bool open_ipc() {
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
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
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (uart_fd < 0) {
        std::cerr << "[ERROR] Failed to open UART: " << strerror(errno) << "\n";
        return -1;
    }
    struct termios tty;
    if (tcgetattr(uart_fd, &tty) != 0)  {
        std::cerr << "[ERROR] tcgetattr failed: " << strerror(errno) << "\n";
        return -1;
    }
    cfsetospeed(&tty, BAUD);
    cfsetispeed(&tty, BAUD);
    cfmakeraw(&tty);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed: " << strerror(errno) << "\n";
        close(uart_fd);
        return -1;
    }
    return 0;
}

void readerLoop() {
    auto next_hb = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[255];
    uint8_t hb_buf[MAVLINK_MAX_PACKET_LEN];
    size_t hb_len;
    struct pollfd pfd{ .fd = uart_fd, .events = POLLIN };

    while (true) {
        // Poll until next heartbeat or incoming data
        auto now = std::chrono::steady_clock::now();
        int timeout = std::chrono::duration_cast<std::chrono::milliseconds>(next_hb - now).count();
        if (timeout < 0) timeout = 0;
        int ret = poll(&pfd, 1, timeout);
        if (ret < 0) {
            std::cerr << "[ERROR] poll: " << strerror(errno) << "\n";
            continue;
        }

        // Send 1Hz GCS heartbeat
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

        if (ret == 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        int len = read(uart_fd, buf, sizeof(buf));
        if (len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                std::cerr << "[ERROR] read failed: " << strerror(errno) << "\n";
            continue;
        } else if (len == 0) {
            continue;
        }

        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                pthread_mutex_lock(&shm_ptr->mutex);
                switch (msg.msgid) {

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

                    case MAVLINK_MSG_ID_MISSION_COUNT: {
                        mavlink_mission_count_t mc;
                        mavlink_msg_mission_count_decode(&msg, &mc);
                        shm_ptr->missionCount = mc.count;
                        shm_ptr->missionState = RUNNING;
                        std::cout << "[DEBUG] Mission count received: " << mc.count << "\n";
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
                    mavlink_mission_request_int_t req;
                    mavlink_msg_mission_request_int_decode(&msg, &req);
                    std::cout << "[DEBUG] Mission request for seq: " << req.seq << "\n";
                    break;
                    }
                    case MAVLINK_MSG_ID_MISSION_CURRENT: {
                        mavlink_mission_current_t mc;
                        mavlink_msg_mission_current_decode(&msg, &mc);
                        std::cout << "[DEBUG] Mission current seq: " << mc.seq << "\n";
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
                        mavlink_mission_item_reached_t mir;
                        mavlink_msg_mission_item_reached_decode(&msg, &mir);
                        std::cout << "[DEBUG] Mission item reached seq: " << mir.seq << "\n";
                        if (mir.seq == static_cast<int>(shm_ptr->missionCount) - 1) {
                            shm_ptr->missionState = NEXT;
                            std::cout << "[DEBUG] Mission completed, state=COMPLETED\n";
                        }
                        break;
                    }

                    default:
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

