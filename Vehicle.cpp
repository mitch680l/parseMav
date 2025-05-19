#include "Vehicle.h"
Vehicle::~Vehicle() = default;


bool Vehicle::validateCommand(const Command& cmd) {
    cmd.printArgs();
    const auto& action = cmd.getAction();
    const auto& args   = cmd.getArgs();

    if (action == "stop") {
        return validateStop();
    } else if (action == "start") {
        return validateStart();
    } else if (action == "move") {
        return validateMove(args);
    } else if (action == "pan") {
        return validatePan(args);
    } else if (action == "tilt") {
        return validateTilt(args);
    } else if (action == "engine") {
        return validateEngine(args);
    } else {
        std::cout << "[" << getName() << "] Unknown action: " << action << "\n";
        return false;
    }
}

bool Vehicle::validateStop() {
    std::cout << "[" << getName() << "] ACK stop\n";
    executeStop();
    return true;
}

bool Vehicle::validateStart() {
    std::cout << "[" << getName() << "] ACK start\n";
    executeStart();
    return true;
}

bool Vehicle::validateMove(const std::vector<std::string>& args) {
    static const std::set<std::string> validDirs = {"forward","backward","left","right"};
    std::string chosen;
    for (const auto& token : args) {
        if (validDirs.count(token)) chosen = token;
    }
    if (chosen.empty()) {
        std::cout << "["<< getName() <<"] Invalid move args\n";
        return false;
    }
    std::cout << "["<< getName() <<"] ACK move "<< chosen <<"\n";
    executeMove(chosen);
    return true;
}

bool Vehicle::validatePan(const std::vector<std::string>& args) {
    float deg = 0.0f;
    bool found = false;
    for (const auto& token : args) {
        try { deg = std::stof(token); found = true; break; }
        catch(...) { continue; }
    }
    if (!found) {
        std::cout << "["<< getName() <<"] Pan command needs a numeric angle\n";
        return false;
    }
    if (deg < 0.0f || deg > 180.0f) {
        std::cout << "["<< getName() <<"] Pan angle out of range (0-180)\n";
        return false;
    }
    std::cout << "["<< getName() <<"] ACK pan "<< deg <<" degrees\n";
    executePan(deg);
    return true;
}

bool Vehicle::validateTilt(const std::vector<std::string>& args) {
    float deg = 0.0f;
    bool found = false;
    for (const auto& token : args) {
        try { deg = std::stof(token); found = true; break; }
        catch(...) { continue; }
    }
    if (!found) {
        std::cout << "["<< getName() <<"] Tilt command needs a numeric angle\n";
        return false;
    }
    if (deg < 0.0f || deg > 180.0f) {
        std::cout << "["<< getName() <<"] Tilt angle out of range (0-180)\n";
        return false;
    }
    std::cout << "["<< getName() <<"] ACK tilt "<< deg <<" degrees\n";
    executeTilt(deg);
    return true;
}

bool Vehicle::validateEngine(const std::vector<std::string>& args) {
    
    return true;
}