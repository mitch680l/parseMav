#include "Plane.h"
void Plane::executeCommand(const Command& cmd) {
    std::cout << "[Plane] Executing command: " << cmd.getAction() << "\n";
}

bool Plane::validateCommand(const Command& cmd) const {
    return false;
}