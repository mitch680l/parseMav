#pragma once


#define SYSID 1 
#define COMPID MAV_COMP_ID_ONBOARD_COMPUTER 


#define TARGET_SYS  1
#define TARGET_COMP 1 

#define UART_DEVICE "/dev/ttyAMA0"
#define ShM_NAME "/mavlink_shm"
#define WAYPOINTS_FILE "../waypoint.csv"
#define BAUD B57600
enum class VehicleType { COPTER, PLANE, ROVER };

enum class FlightMode {
    MANUAL, STABILIZE, ACRO,
    GUIDED, AUTO, RTL, LOITER,
};