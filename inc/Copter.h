#pragma once
#include "Vehicle.h"
#include <iostream>

class Copter : public Vehicle {
public:
    std::string getName() const override { return "Copter"; }
    void executeStop() override;
    void executeStart() override;
    void executeMove(std::vector<Waypoint> waypoints) override;
    void executePan(float deg) override;
    void executeTilt(float deg) override;
    void executeTurn(float yaw, int direction, bool relative) override;
    void executeAdvance(float lat, float lng, float alt) override;
    void executeMission(std::string missionCommand) override;
    void executeEngine(const std::string& command) override;
    void executeArm(std::string command) override;
};