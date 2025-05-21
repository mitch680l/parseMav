#include "reader.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
int main() {
    if (!open_ipc()) {
        std::cerr << "Error: failed to open shared memory\n";
        return 1;
    }
    if (!init_cmd_uart()) {
        std::cerr << "Error: failed to open UART\n";
        return 2;
    }
    // now getTelem()/getBearing() will see the reader’s data
    while (true) {
        SendHeartbeat();
        double lat, lon, alt, yaw;
        getTelem(&lat, &lon, &alt);
        getBearing(&yaw);
        std::cout
            << "Telem: lat=" << lat
            << ", lon=" << lon
            << ", alt=" << alt << " m"
            << ", yaw=" << yaw << "°\n"
            << std::flush;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(200));
    }
    return 0;
}