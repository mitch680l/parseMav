#pragma once
#include "Vehicle.h"
#include <iostream>

class Plane : public Vehicle {
public:
    std::string getName() const override { return "Plane"; }
    void executeCommand(const Command& cmd) override;
};