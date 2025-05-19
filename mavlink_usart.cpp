#include "mavlink_usart.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include "c_library_v2-master/ardupilotmega/mavlink.h"

static int uart_fd = -1;

int init_uart(void) {
    const char *device = "/dev/serial0";
    uart_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0) {
        perror("open serial");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(uart_fd, &tty) != 0) {
        perror("tcgetattr");
        close(uart_fd);
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(uart_fd);
        return -1;
    }

    return 0;
}

void SendHeartbeat(void) {
    if (uart_fd < 0) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        1,              // sysid
        191,            // compid
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0,
        MAV_STATE_ACTIVE
    );

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(uart_fd, buf, len);
}

void SendServo(uint8_t servo, uint16_t pwm) {
    //if (uart_fd < 0) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_servo_output_raw_pack(
        1,             
        191,            
        &msg,
        0,              
        0,              
        servo,          
        pwm,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0     
    );

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(uart_fd, buf, len);
}