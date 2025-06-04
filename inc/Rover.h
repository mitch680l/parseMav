#pragma once
#include "Vehicle.h"
#include <iostream>

class Rover : public Vehicle {
public:
    Rover();
    ~Rover() override;
    std::string getName() const override;
    enum Mode {
        MANUAL = 0,
        ACRO = 1,
        STEERING = 2,
        HOLD = 3,
        LOITER = 4,
        FOLLOW = 6,
        SIMPLE = 7,
        AUTO = 10,
        RTL = 11,
        GUIDED = 15,
        SMART_RTL = 16,
    };
    std::string getModeName(uint32_t mode) const override {
        switch (mode) {
            case MANUAL: return "MANUAL";
            case ACRO: return "ACRO";
            case STEERING: return "STEERING";
            case HOLD: return "HOLD";
            case LOITER: return "LOITER";
            case FOLLOW: return "FOLLOW";
            case SIMPLE: return "SIMPLE";
            case AUTO: return "AUTO";
            case RTL: return "RTL";
            case GUIDED: return "GUIDED";
            case SMART_RTL: return "SMART_RTL";
            default: return "UNKNOWN";
        }
    }
private:
    // Action-specific executors
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
    void executeMode(const std::string& mode, const std::string& type) override;

    
};