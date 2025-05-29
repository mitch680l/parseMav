#pragma once


#define SYSID 1 
#define COMPID MAV_COMP_ID_MISSIONPLANNER


#define TARGET_SYS  1
#define TARGET_COMP 1 
#define DEBUG_SEND true

#define INPUT_TIMEOUT 1000
#define VEHICLE_NAME "rover 67"
#define VEHICLE_TYPE "rover"
#define ENABLE_GLOBAL_KEYWORDS false
#define ENABLE_VEHICLE_TYPE_KEYWORDS false
#define ENABLE_CALLSIGN_KEYWORDS true
#define REQUIRE_OVER true

#define UART_DEVICE "/dev/ttyAMA0"
#define ShM_NAME "/mavlink_shm"
#define FIFO_PATH "/tmp/speech_pipe"
#define WAYPOINTS_FILE "../waypoints.csv"
#define ACTIONS_FILE "../actions.cfg"
#define BAUD B57600
enum class VehicleType { COPTER, PLANE, ROVER };

enum class FlightMode {
    MANUAL, STABILIZE, ACRO,
    GUIDED, AUTO, RTL, LOITER,
};

enum MissionExecState {
    IDLE = 0,
    RUNNING,
    NEXT,
    COMPLETED
};
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
    uint8_t           current_seq;
};

extern SharedTelem* shm_ptr;

