#include "Rover.h"
#include <iostream>
#include <cmath>
#include <set>
#include "helper.h"

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

std::string Rover::getName() const {
    return "Rover";
  }