#include <iostream>
#include <thread>
#include <chrono>
#include "reader.h"
#include "mavlink_usart.h"

int main() {
    if (!open_ipc()) {
        std::cerr << "Error: failed to open shared memory\n";
        return 1;
    }
    if (!init_cmd_uart()) {
        std::cerr << "Error: failed to open UART\n";
        return 2;
    }
    
    std::vector<MissionItem> missionPlan;
    double lat1 = 38.972387;
    double lon1 = -95.233159;
    double alt = 0;
    getTelem(&lat1, &lon1, &alt);
    std::cout << "Current position: " << lat1 << ", " << lon1 << ", " << alt << std::endl;
    double distance = 5000;
    double bearing = 90;
    getBearing(&bearing);
    std::cout << "Current bearing: " << bearing << std::endl;
    SendServo(7,1000);
    SendServo(8,1000);
    double lat2, lon2;
    vincentyDirect(lat1, lon1, bearing, distance, lat2, lon2);
    std::cout << "Vincenty direct result1: " << lat2 << ", " << lon2 << std::endl;
    //clearMission();
    appendWaypoint(lat2,lon2,alt, missionPlan);
    double distance2 = 5000;
    double bearing2 = 180;
    vincentyDirect(lat1, lon1, bearing2, distance2, lat2, lon2);
    appendWaypoint(lat2,lon2,alt, missionPlan);
    distance2 = 50000;
    bearing2 = 90;
    vincentyDirect(lat1, lon1, bearing2, distance2, lat2, lon2);
    appendWaypoint(lat2,lon2,alt, missionPlan);
    
    throttle(1);
    std::cout << "Throttle Armed" << std::endl;
    //setFlightMode(FlightMode::AUTO);
    std::cout << "Flight mode set to AUTO" << std::endl;
    std::cout << getMissionSize(missionPlan) << " waypoints in the mission plan" << std::endl;
    distance2 = 10000;
    bearing2 = 270;
    vincentyDirect(lat1, lon1, bearing2, distance2, lat2, lon2);
    appendWaypoint(lat2,lon2,alt, missionPlan);
    

    std::cout << "Simulating mission Current = 1" << std::endl;
    //setCurrent(1);
    startMissionUpload(missionPlan);
    int count = 0;
    while (count < 3) {
        count++;
    }
    //throttle(0);
    //std::cout << "Throttle armed" << std::endl;
    //setFlightMode(FlightMode::MANUAL);
    //std::cout << "Flight mode set to GUIDED" << std::endl;
    return 0;
}