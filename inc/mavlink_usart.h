#pragma once

#include <cstdint>
#include "c_library_v2-master/ardupilotmega/mavlink.h"
#include "config.h"
#include "helper.h"
#ifdef __cplusplus
extern "C" {
#endif
/*
This file contains the functions for sending sending mavlink messages to the autopilot.
It also sets up a UART connection to the autopilot that allows for sending only.
*/

//Standard tx/rx implmentation for rpi(9600 baud for now).
//int init_uart(void);
extern int cmd_fd;

bool init_cmd_uart(void);
/*
Send a heartbeat message to the autopilot. Identifies this system as 191 "onboard computer" see config.h for definition
*/
void SendHeartbeat(void);

/*
servo: channel number (based on where the servo is connected)
pwm: pulse width modulation value (0-2000) however realistically
      it should be between 1000 and 2000 for most servos.
      1000 is the minimum and 2000 is the maximum.
*/
void SendServo(uint8_t servo, uint16_t pwm);

/*
Wrapper for rpi write function
*/
void Send(int fd, const uint8_t *buf, size_t len);
//void Read(int uart_fd, uint8_t *buf, size_t len);

/*
Add a waypoint to the mission
*/
void appendWaypoint(int32_t lat, int32_t lon, float alt);

/*
Add a turn to the mission (change yaw)
*/
void appendTurn(float yaw_deg, float yaw_rate, int8_t direction, bool relative);

/*
clear the missions
*/
void clearMission();

/*
start the mission(only for plane)
*/
void startMission();

/*
arm/disarm the vehicle
*/
void throttle(int arm);

/*

*/
bool setFlightMode(FlightMode fm);








#ifdef __cplusplus
}
#endif