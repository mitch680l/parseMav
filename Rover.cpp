#include "Rover.h"
void Rover::executeCommand(const Command& cmd) {
    std::cout << "[Rover] Executing command: " << cmd.getAction() << "\n";
}
