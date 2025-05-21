#include "SpeechInterpreter.h"
#include <iostream>
#include <thread>
#include <chrono>
#include "mavlink_usart.h"
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
    SpeechInterpreter interpreter("/tmp/speech_pipe");
    interpreter.run();
    return 0;
}