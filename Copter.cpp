#include "Copter.h"
void Copter::executeCommand(const Command& cmd) {
    std::cout << "[Copter] Executing command: " << cmd.getAction() << "\n";
}
bool Copter::validateCommand(const Command& cmd) const {
    return false;
}