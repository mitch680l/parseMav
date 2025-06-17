#pragma once
#include <atomic>
//mavlink indenfication
#define SYSID 1 
#define COMPID MAV_COMP_ID_MISSIONPLANNER
#define TARGET_SYS  1 //target system ID for MAVLink messages(i.e the autopilot)
#define TARGET_COMP 1 // target component ID for MAVLink messages(i.e the autopilot)



//parser options
#define LAUNCH_READER true //automatically launch the reader process on startup using fork and exec
#define INPUT_TIMEOUT 3000 //how long parser waits for input before trying to execute a command (in milliseconds)
#define VEHICLE_NAME "rover 67" //callsign for the vehicle
#define VEHICLE_TYPE "rover" //type of vehicle (copter, plane, rover, etc)
#define ENABLE_GLOBAL_KEYWORDS false //enable global keywords like "all", "any", "none", etc
#define ENABLE_VEHICLE_TYPE_KEYWORDS false //enable vehicle type keywords like "copter", "plane", "rover", etc
#define ENABLE_CALLSIGN_KEYWORDS true //enable callsign keywords like "rover 67", "copter 42", etc
#define REQUIRE_OVER false //require "over" keyword for commands that require it (e.g. "rover 67 advance 100 meters over"). Without this, commands will wait for timeout and try to execute.
#define ENABLE_TOGGLE_MODE true         // Allow "callsign on/off" activation toggle
#define ENABLE_NO_ID_MODE  false    // Accept commands without any identification


//perphial settings
#define UART_DEVICE "/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_280028000C51313132383631-if00" // OS resource for uart connection
//sudo ls /dev/serial/by-id/
//serial 0

#define ShM_NAME "/mavlink_shm" // shared memory name for IPC
#define FIFO_PATH "/tmp/speech_pipe" // path to the named pipe for speech input *IF PROGRAM IS NOT WORKING, ITS POSSIBLE THIS FILE GOT DELETED or CORRUPTED* (use mkfifo /tmp/speech_pipe)
#define WAYPOINTS_FILE "../waypoints.csv" // path to the waypoints file
#define ACTIONS_FILE "../actions.cfg" // path to the actions file
#define BAUD B57600 // baud rate for UART connection
#define PAN_CHANNEL 6 //channel for pan servo
#define TILT_CHANNEL 7 //channel for tilt servo
#define PWM_MIN 500 //minimum PWM value for servos
#define PWM_MAX 2500 //maximum PWM value for servos
#define SERVO_RANGE_TILT 180.0 //servo range in degrees
#define SERVO_RANGE_PAN 360.0 //servo range in degrees
#define SERVO_PAN_START 1500 //start position for pan servo
#define SERVO_TILT_START 1500 //start position for tilt servo
inline double modifier_tilt = (PWM_MAX - PWM_MIN) / SERVO_RANGE_TILT; //ratio of pwm to degrees(example max = 2500, min = 500, range = 180, modifier = (2500-500)/180 = 11.11) so for every degree, its 11.11 pwm
inline double modifier_pan = (PWM_MAX - PWM_MIN) / SERVO_RANGE_PAN; //ratio of pwm to degrees(example max = 2500, min = 500, range = 360, modifier = (2500-500)/360 = 5.56) so for every degree, its 5.56 pwm
//param set SERIAL0_BAUD 57600
//param show SERIAL0_BAUD

//flags for debug outputs
#define DEBUG_SEND 1
#define DEBUG_MONITOR_STATE true
#define DEBUG_SHARED_TELEM true
#define DEBUG_PREARM_MESSAGES 0
#define DEBUG_POSITION_MESSAGES     0
#define DEBUG_ATTITUDE_MESSAGES     0
#define DEBUG_SYSTEM_MESSAGES       0
#define DEBUG_MISSION_MESSAGES      1
#define DEBUG_SENSOR_MESSAGES       0
#define DEBUG_RC_SERVO_MESSAGES     0
#define DEBUG_STATUS_MESSAGES       0
#define DEBUG_COMMAND_MESSAGES      1
#define DEBUG_PARAMETER_MESSAGES    0
#define DEBUG_NAVIGATION_MESSAGES   0
#define DEBUG_MISC_MESSAGES        0
#define DEBUG_RC_MESSAGES          0
#define DEBUG_SIM_MESSAGES          0
#define SIM_MODE 1


//operation settings
#define AUTO_START_MISSION false //automatically start mission on startup (given that a mission is loaded and its safe to do so)
#define AUTO_LOAD_MISSION false //automatically load missions when items are added to the mission queue
#define MISSION_MODE false //preference to add commands to a mission queue versus executing them immediately. I.e. advance 100 meters is a target_global versus a waypoint(TO DO(MAYBE))
enum class VehicleType { COPTER, PLANE, ROVER };



enum MissionExecState {
    IDLE = 0,
    RUNNING,
    NEXT,
    COMPLETED
};
enum MissionState {
    MISSION_UNKNOWN = 0,
    MISSION_NO_MISSION = 1,
    MISSION_NOT_STARTED = 2,
    MISSION_ACTIVE = 3,
    MISSION_PAUSED = 4,
    MISSION_COMPLETED = 5
};
/*
This is the shared memory structure that is used to share data between the reader and the main program.
*/
struct SharedTelem {
    pthread_mutex_t   mutex;
    double            lat, lon, alt;
    double            yaw_deg;
    uint64_t          seq;            
    MissionExecState  missionUploadState;   
    uint32_t          missionCount;
    uint32_t          missionCountAutopilot;   
    uint8_t           current_seq;
    bool              armed;
    int               mode;
    MissionState      mission_state;
    bool              requestParams;
};

struct VehicleFlags {
    static inline std::atomic<bool> missionRunning{false};
    static inline std::atomic<bool> missionNeedsStart{false}; 
    static inline std::atomic<bool> missionPaused{false};
    static inline std::atomic<bool> monitorRunning{false};
    static inline std::atomic<bool> engine{false};
    static inline std::atomic<bool> brake{false};
    static inline std::atomic<bool> shouldStart{false};
    static inline std::atomic<bool> shouldLoad{false};
    static inline std::atomic<bool> newItems{false};
    static inline std::atomic<bool> armState{false};
    static inline std::atomic<bool> safeMove{false};
};

extern SharedTelem* shm_ptr;

struct HealthStatus {
    bool ipcAlive        = false;
    bool uartAlive       = false;
    bool readerAlive     = false;
    int uartFailCount    = 0;
    int ipcFailCount     = 0;
    int readerFailCount  = 0;
};


