#pragma once
#include "Vehicle.h"
#include <iostream>

class Rover : public Vehicle {
public:
    std::string getName() const override { return "Rover"; }
    void executeCommand(const Command& cmd) override;
    bool validateCommand(const Command& cmd) const override;

};


