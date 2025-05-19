#include <iostream>
#include <thread>
#include <chrono>

#include "mavlink_usart.h"

int main() {
    if (init_uart() != 0) {
        std::cerr << "Failed to initialize UART" << std::endl;
        return 1;
    }
    else {
        std::cout << "UART opened OK" << std::endl;
    }

    while (true) {
        SendHeartbeat();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Heartbeat sent" << std::endl;
    }

    return 0;
}