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

//Mission plan item
struct MissionItem {
        mavlink_mission_item_int_t msg;
};

struct Waypoint {
    double lat;
    double lon;
    double alt;
};




/*
init the UART (write only) for the main process
*/
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
Wrapper for rpi write function, has multple debug options
*/
void Send(int fd, const uint8_t *buf, size_t len);
//void Read(int uart_fd, uint8_t *buf, size_t len);

/*
clear the missions from autopilot
*/
void clearMission();

/*
arm/disarm the vehicle(maybe rename this to arm?)
*/
void throttle(int arm);

/*
Set mode of the vehicle
*/
bool setFlightMode(int mode);

/*
Add a waypoint to the mission queue.
*/
void appendWaypoint(double lat, double lon, float alt, std::vector<MissionItem>& missionPlan);

/*
Add a turn to the mission queue
*/
void appendTurn(float yaw_deg, float yaw_rate, int8_t direction, bool relative, std::vector<MissionItem>& missionPlan);

/*
begin the mission upload process.
*/
void startMissionUpload(std::vector<MissionItem>& missionPlan);

int getMissionSize(std::vector<MissionItem>& missionPlan);
void setMissionCurrent(uint16_t seq);

void sendMissionStartCommand();
void sendMissionStopCommand();
void sendMissionResumeCommand();
void sendMissionPauseCommand();

#ifdef __cplusplus
}
#endif