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
    std::cout << "Plane stopping\n";

}

void Plane::executeStart() {
    std::cout << "Plane starting\n";
}

void Plane::executeMove(const std::string& dir) {
    std::cout << "Plane move " << dir << "\n";
}

void Plane::executePan(float deg) {
    std::cout << "Plane pan " << deg << "\n";
}

void Plane::executeTilt(float deg) {
    std::cout << "Plane tilt " << deg << "\n";
}
