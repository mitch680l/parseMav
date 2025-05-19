#include "SpeechInterpreter.h"
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
    SpeechInterpreter interpreter("/tmp/speech_pipe");
    interpreter.run();
    return 0;
}