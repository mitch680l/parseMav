#pragma once

#define SYSID 1 
#define COMPID MAV_COMP_ID_ONBOARD_COMPUTER 
#define TARGET_SYS  1
#define TARGET_COMP 1 

enum class VehicleType { COPTER, PLANE, ROVER };

enum class FlightMode {
    MANUAL, STABILIZE, ACRO,
    GUIDED, AUTO, RTL, LOITER,
};