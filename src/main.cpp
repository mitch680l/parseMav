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
    SpeechInterpreter alphaRover(VEHICLE_TYPE, VEHICLE_NAME, FIFO_PATH, ACTIONS_FILE, INPUT_TIMEOUT);
    alphaRover.run();

    return 0;
}