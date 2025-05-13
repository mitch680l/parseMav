#include "Rover.h"
void Rover::executeCommand(const Command& cmd) {
    std::cout << "[Rover] Executing command: " << cmd.getAction() << "\n";

    if (cmd.getAction() == "stop") {
        std::cout << "Set throttle to idle " << "\n";
        std::cout << "Apply full brake " << "\n";
        std::cout << "Set gear to neutral " << "\n";
        std::cout << "Ignition ON " << "\n";
    }
    else {
        std::cout << "[Rover] Unknown command: " << cmd.getAction() << "\n";
    }
}

bool Rover::validateCommand(const Command& cmd) const {
    const auto& args = cmd.getArgs();

    if (cmd.getAction() == "stop") {
        std::cout << "ACK stop " << "\n";
        return args.empty();
    }
    if (cmd.getAction() == "start") {
        std::cout << "ACK start " << "\n";
        return args.empty();
    }
    if (cmd.getAction() == "move") {
        std::cout << "ACK  " << "\n";
        if (args.empty()) return false;
        static const std::set<std::string> validDirections = {
            "forward", "backward", "left", "right"
        };
        return validDirections.count(args[0]) > 0;
    }

    return false;
}