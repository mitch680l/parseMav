#include "Rover.h"
#include <iostream>
#include <cmath>
#include <set>


Rover::Rover() = default;
Rover::~Rover() = default;

void Rover::executeStop() {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    flags.engine = false;
    std::cout << "Executing stop: idle throttle, full brake, neutral gear, ignition OFF\n";
}

void Rover::executeStart() {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    flags.engine = true;
    std::cout << "Executing start: ignition ON, system ready\n";
}

void Rover::executeMove(std::vector<Waypoint> waypoints) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    for (const auto& wp : waypoints) {
        appendWaypoint(wp.lat, wp.lon, wp.alt, missionPlan);
    }
    std::cout << "Executing move: added " << waypoints.size() << " waypoints to mission plan\n";
}

void Rover::executePan(float angleDeg) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    int pwm = angleToPwm(angleDeg, modifier_pan);
    std::cout << "Executing pan to " << angleDeg << "° -> PWM " << pwm << "\n";
    SendServo(PAN_CHANNEL,pwm);

}

void Rover::executeTilt(float angleDeg) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    int pwm = angleToPwm(angleDeg, modifier_tilt);
    std::cout << "Executing tilt to " << angleDeg << "° -> PWM " << pwm << "\n";
    SendServo(TILT_CHANNEL,pwm);
}

void Rover::executeTurn(float yaw, int direction, bool relative) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    appendTurn(yaw,30.0f, direction, relative, missionPlan);
    std::cout << "Executing turn: yaw " << yaw << "° direction " << (direction == 1 ? "right" : "left") << (relative ? " (relative)" : "") << "\n";
}
void Rover::executeAdvance(float lat, float lng, float alt) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    appendWaypoint(lat, lng, alt, missionPlan);
    std::cout << "Executing advance: moving to lat " << lat << ", lng " << lng << ", alt " << alt << "\n";
}
std::string Rover::getName() const {
    return "Rover";
  }

void Rover::executeMission(std::string missionCommand) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    if (missionCommand == "start") {
        flags.shouldStart = true;
    } else if (missionCommand == "stop") {
        sendMissionStopCommand();
    } else if (missionCommand == "pause") {
        sendMissionPauseCommand();
    } else if (missionCommand == "resume") {
        sendMissionResumeCommand();
    } else if (missionCommand == "details") {
        showMissionDetails();
    } else if (missionCommand == "clear") {
        missionPlan.clear(); //clear mission plan vector
        std::cout << "Executing mission clear: cleared all waypoints and commands\n";
        clearMission(); //clear autopilot
    } else if (missionCommand == "upload") {
        flags.shouldLoad = true;
    }
    else {
        std::cout << "Error: Unknown mission command " << missionCommand << "'\n";
    }
}

void Rover::executeEngine(const std::string& command) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    if (command == "start") {
        flags.engine = true;
        std::cout << "Executing engine command: ON\n";
    } else if (command == "stop") {
        flags.engine = false;
        std::cout << "Executing engine command: OFF\n";
    } else {
        std::cout << "Error: Unknown engine command '" << command << "'\n";
    }
}

void Rover::executeArm(std::string command) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    if (command == "arm") {
        throttle(1);
        std::cout << "Executing arm command: armed\n";
    } else if (command == "disarm") {
        throttle(0);
        std::cout << "Executing disarm command: disarmed\n";
    } else {
        std::cout << "Error: Unknown arm command '" << command << "'\n";
    }
}

void Rover::executeMode(const std::string& mode, const std::string& type) {
    std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
    if (type == "flight") {
        std::cout << "Executing flight mode change to '" << mode << "'\n";
        if (mode == "manual") {
            setFlightMode(Mode::MANUAL);
        } else if (mode == "acro") {
            setFlightMode(Mode::ACRO);
        } else if (mode == "steering") {
            setFlightMode(Mode::STEERING);
        } else if (mode == "hold") {
            setFlightMode(Mode::HOLD);
        } else if (mode == "loiter") {
            setFlightMode(Mode::LOITER);
        } else if (mode == "follow") {
            setFlightMode(Mode::FOLLOW);
        } else if (mode == "simple") {
            setFlightMode(Mode::SIMPLE);
        } else if (mode == "auto") {
            setFlightMode(Mode::AUTO);
        } else if (mode == "rtl") {
            setFlightMode(Mode::RTL);
        } else if (mode == "guided") {
            setFlightMode(Mode::GUIDED);
        } else if (mode == "smart_rtl") {
            setFlightMode(Mode::SMART_RTL);
        } else {
            std::cout << "Error: Unknown flight mode '" << mode << "'\n";
        }
    }
    else {
        std::cout << "Other mode changes unavailable atm" << std::endl;
    }
}