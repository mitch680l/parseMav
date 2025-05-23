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

    double lat1 = 38.972387;
    double lon1 = -95.233159;
    double alt = 0;
    getTelem(&lat1, &lon1, &alt);
    std::cout << "Current position: " << lat1 << ", " << lon1 << ", " << alt << std::endl;
    double distance = 100000;
    double bearing = 90;
    getBearing(&bearing);
    std::cout << "Current bearing: " << bearing << std::endl;

    double lat2, lon2;
    vincentyDirect(lat1, lon1, bearing, distance, lat2, lon2);
    std::cout << "Vincenty direct result: " << lat2 << ", " << lon2 << std::endl;
    SendHeartbeat();
    throttle(1);
    std::cout << "Throttle Armed" << std::endl;
    setFlightMode(FlightMode::AUTO);
    appendWaypoint(lat2,lon1,alt);

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