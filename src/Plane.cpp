#include "Plane.h"


#include "Plane.h"
#include <iostream>

void Plane::executeTurn(float yaw, int direction, bool relative) {
    std::cout << "Plane turning yaw=" << yaw
              << " dir=" << direction
              << " relative=" << relative << "\n";
}

void Plane::executeAdvance(float lat, float lng, float alt) {
    std::cout << "Plane advance to lat=" << lat
              << " lng=" << lng
              << " alt=" << alt << "\n";
}

void Plane::executeStop() {
    //engineOn = false;
    std::cout << "Plane stopping\n";

}

void Plane::executeStart() {
    //engineOn = true;
    std::cout << "Plane starting\n";
}

void Plane::executeMove(std::vector<Waypoint> waypoints) {
    std::cout << "Plane move " << "\n";
}

void Plane::executePan(float deg) {
    std::cout << "Plane pan " << deg << "\n";
}

void Plane::executeTilt(float deg) {
    std::cout << "Plane tilt " << deg << "\n";
}

void Plane::executeMission(std::string missionCommand) {
    std::cout << "Plane executing mission: " << missionCommand << "\n";
}

void Plane::executeEngine(const std::string& command) {
    std::cout << "Plane executing engine command: " << command << "\n";
}
void Plane::executeArm(std::string action) {
    std::cout << "Plane executing arm action: " << action << "\n";
}

void Plane::executeMode(const std::string& mode, const std::string& type) {
    std::cout << "Plane executing mode change to: " << mode << " of type: " << type << "\n";
}