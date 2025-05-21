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

void Copter::executeMove(const std::string& dir) {
    std::cout << "Copter move " << dir << "\n";
}

void Copter::executePan(float deg) {
    std::cout << "Copter pan " << deg << "\n";
}

void Copter::executeTilt(float deg) {
    std::cout << "Copter tilt " << deg << "\n";
}