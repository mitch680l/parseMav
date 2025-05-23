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

/*
These are the states for the mission upload handshake process.
*To-do: make this more clear and robust. Its a bit of a mess right now.
ExecState is the state of the mission execution(autopilot side/reader).
UploadState is the state of the mission upload(writer side).
*/
enum MissionExecState {
    IDLE = 0,
    RUNNING,
    NEXT,
    COMPLETED
};
enum MissionUploadState { UP_IDLE, UP_RUNNING, UP_NEXT, UP_COMPLETED };

/*
This is the shared memory structure that is used to share data between the reader and the main program.
*/
struct SharedTelem {
    pthread_mutex_t   mutex;
    double            lat, lon, alt;
    double            yaw_deg;
    uint64_t          seq;            
    MissionExecState  missionState;   
    uint32_t          missionCount;   
};

//uart file descriptor/os resource
int cmd_fd = -1; 
//shared memory file descriptor(call open_ipc() to to allocate this pointer).
static SharedTelem* shm_ptr = nullptr;

//Mission plan vector
typedef std::vector<uint8_t> ByteBuffer;
struct MissionItem {
    ByteBuffer buf;
};
static std::vector<MissionItem> missionPlan;



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
start the mission(only for plane)
*/
void startMission();

/*
arm/disarm the vehicle(maybe rename this to arm?)
*/
void throttle(int arm);

/*
Set mode of the vehicle
*/
bool setFlightMode(FlightMode fm);

/*
Add a waypoint to the mission queue.
*/
void appendWaypoint(double lat, double lon, float alt);

/*
Add a turn to the mission queue
*/
void appendTurn(float yaw_deg, float yaw_rate = 30.0f, int8_t direction = 1, bool relative = false);

/*
begin the mission upload process.
*/
void startMissionUpload();


/*
Send an individual mission item to the autopilot.(when its ready)
*/
void sendMissionItem(size_t seq);



#ifdef __cplusplus
}
#endif