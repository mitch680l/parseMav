#include "Copter.h"

void Copter::executeTurn(float yaw, int direction, bool relative) {
    std::cout << "Copter turning yaw=" << yaw
              << " dir=" << direction
              << " relative=" << relative << "\n";
}

void Copter::executeAdvance(float lat, float lng, float alt) {
    std::cout << "Copter advance to lat=" << lat
              << " lng=" << lng
              << " alt=" << alt << "\n";
}

void Copter::executeStop() {
    std::cout << "Copter stopping\n";

}

void Copter::executeStart() {
    std::cout << "Copter starting\n";
}

void Copter::executeMove(std::vector<Waypoint> waypoints) {
    std::cout << "Copter move " << "\n";
}

void Copter::executePan(float deg) {
    std::cout << "Copter pan " << deg << "\n";
}

void Copter::executeTilt(float deg) {
    std::cout << "Copter tilt " << deg << "\n";
}

void Copter::executeMission(std::string missionCommand) {
    std::cout << "Copter executing mission command: " << missionCommand << "\n";
}

void Copter::executeEngine(const std::string& command) {
    std::cout << "Copter executing engine command: " << command << "\n";
}

void Copter::executeArm(std::string command) {
    std::cout << "Copter executing arm action: " << command << "\n";
}