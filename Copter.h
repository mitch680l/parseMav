#pragma once
#include "Vehicle.h"
#include <iostream>

class Copter : public Vehicle {
public:
    std::string getName() const override { return "Copter"; }
    bool validateCommand(const Command& cmd)  override;
};