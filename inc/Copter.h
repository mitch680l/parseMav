#pragma once
#include "Vehicle.h"
#include <iostream>

class Copter : public Vehicle {
public:
    enum Mode {
    STABILIZE       = 0,
    ACRO            = 1,
    ALT_HOLD        = 2,
    AUTO            = 3,
    GUIDED          = 4,
    LOITER          = 5,
    RTL             = 6,
    CIRCLE          = 7,
    LAND            = 9,
    DRIFT           = 11,
    SPORT           = 13,
    FLIP            = 14,
    AUTOTUNE        = 15,
    POSHOLD         = 16,
    BRAKE           = 17,
    THROW           = 18,
    AVOID_ADSB      = 19,
    GUIDED_NOGPS    = 20,
    SMART_RTL       = 21,
    FLOWHOLD        = 22,
    FOLLOW          = 23,
    ZIGZAG          = 24,
    SYSTEMID        = 25,
    AUTOROTATE      = 26,
    AUTO_RTL        = 27
    };
    std::string getModeName(uint32_t mode) const override {
        switch (mode) {
            case STABILIZE: return "STABILIZE";
            case ACRO: return "ACRO";
            case ALT_HOLD: return "ALT_HOLD";
            case AUTO: return "AUTO";
            case GUIDED: return "GUIDED";
            case LOITER: return "LOITER";
            case RTL: return "RTL";
            case CIRCLE: return "CIRCLE";
            case LAND: return "LAND";
            case DRIFT: return "DRIFT";
            case SPORT: return "SPORT";
            case FLIP: return "FLIP";
            case AUTOTUNE: return "AUTOTUNE";
            case POSHOLD: return "POSHOLD";
            case BRAKE: return "BRAKE";
            case THROW: return "THROW";
            case AVOID_ADSB: return "AVOID_ADSB";
            case GUIDED_NOGPS: return "GUIDED_NOGPS";
            case SMART_RTL: return "SMART_RTL";
            case FLOWHOLD: return "FLOWHOLD";
            case FOLLOW: return "FOLLOW";
            case ZIGZAG: return "ZIGZAG";
            case SYSTEMID: return "SYSTEMID";
            case AUTOROTATE: return "AUTOROTATE";
            case AUTO_RTL: return "AUTO_RTL";
            default: return "UNKNOWN MODE"; 
        }
    }
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
    void executeMode(const std::string& mode, const std::string& type) override;
protected:
    
};