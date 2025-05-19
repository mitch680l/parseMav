#pragma once
#include "Vehicle.h"
#include <iostream>

class Plane : public Vehicle {
public:
    std::string getName() const override { return "Plane"; };
    bool validateCommand(const Command& cmd) override;
};