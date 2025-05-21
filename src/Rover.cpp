#include "Rover.h"
#include <iostream>
#include <cmath>
#include <set>


Rover::Rover() = default;
Rover::~Rover() = default;

void Rover::executeStop() {
    std::cout << "Executing stop: idle throttle, full brake, neutral gear, ignition ON\n";
}

void Rover::executeStart() {
    std::cout << "Executing start: ignition ON, system ready\n";
}

void Rover::executeMove(const std::string& direction) {
    std::cout << "Executing move: " << direction << "\n";
}

void Rover::executePan(float angleDeg) {
    int pwm = angleToPwm(angleDeg);
    std::cout << "Executing pan to " << angleDeg << "° -> PWM " << pwm << "\n";\
    SendServo(6,pwm);

}

void Rover::executeTilt(float angleDeg) {
    int pwm = angleToPwm(angleDeg);
    std::cout << "Executing tilt to " << angleDeg << "° -> PWM " << pwm << "\n";
    SendServo(7,pwm);
}

void Rover::executeTurn(float yaw, int direction, bool relative) {
    appendTurn(yaw,30.0f, direction, relative);
    std::cout << "Executing turn: yaw " << yaw << "° direction " << (direction == 1 ? "right" : "left") << (relative ? " (relative)" : "") << "\n";
}
void Rover::executeAdvance(float lat, float lng, float alt) {
    appendWaypoint(lat, lng, alt);
    std::cout << "Executing advance: moving to lat " << lat << ", lng " << lng << ", alt " << alt << "\n";
}
std::string Rover::getName() const {
    return "Rover";
  }