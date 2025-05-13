#pragma once
#include "Vehicle.h"
#include <iostream>

class Copter : public Vehicle {
public:
    std::string getName() const override { return "Copter"; }
    void executeCommand(const Command& cmd) override;
    bool validateCommand(const Command& cmd) const override;
};