#pragma once
#include "Vehicle.h"
#include <iostream>

class Plane : public Vehicle {
public:
    std::string getName() const override { return "Plane"; };
    void executeStop() override;
    void executeStart() override;
    void executeMove(const std::string& dir) override;
    void executePan(float deg) override;
    void executeTilt(float deg) override;
    void executeTurn(float yaw, int direction, bool relative) override;
    void executeAdvance(float lat, float lng, float alt) override;
};