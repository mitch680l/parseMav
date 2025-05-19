#pragma once
#include "Vehicle.h"
#include <iostream>

class Rover : public Vehicle {
public:
    Rover();
    ~Rover() override;
    std::string getName() const override;
private:

    // Action-specific executors
    void executeStop() override;
    void executeStart() override;
    void executeMove(const std::string& dir) override;
    void executePan(float deg) override;
    void executeTilt(float deg) override;

};