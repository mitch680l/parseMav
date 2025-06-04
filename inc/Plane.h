#pragma once
#include "Vehicle.h"
#include <iostream>

class Plane : public Vehicle {
public:
    enum Mode {
        MANUAL         = 0,
        CIRCLE         = 1,
        STABILIZE      = 2,
        TRAINING       = 3,
        ACRO           = 4,
        FBWA           = 5,
        FBWB           = 6,
        CRUISE         = 7,
        AUTOTUNE       = 8,
        AUTO           = 10,
        RTL            = 11,
        LOITER         = 12,
        TAKEOFF        = 13,
        AVOID_ADSB     = 14,
        GUIDED         = 15,
        INITIALISING   = 16,
        QSTABILIZE     = 17,
        QHOVER         = 18,
        QLOITER        = 19,
        QLAND          = 20,
        QRTL           = 21,
        QAUTOTUNE      = 22,
        QACRO          = 23,
        THERMAL        = 24
    };
    std::string getModeName(uint32_t mode) const override {
        switch (mode) {
            case MANUAL: return "MANUAL";
            case CIRCLE: return "CIRCLE";
            case STABILIZE: return "STABILIZE";
            case TRAINING: return "TRAINING";
            case ACRO: return "ACRO";
            case FBWA: return "FBWA";
            case FBWB: return "FBWB";
            case CRUISE: return "CRUISE";
            case AUTOTUNE: return "AUTOTUNE";
            case AUTO: return "AUTO";
            case RTL: return "RTL";
            case LOITER: return "LOITER";
            case TAKEOFF: return "TAKEOFF";
            case AVOID_ADSB: return "AVOID_ADSB";
            case GUIDED: return "GUIDED";
            case INITIALISING: return "INITIALISING";
            case QSTABILIZE: return "QSTABILIZE";
            case QHOVER: return "QHOVER";
            case QLOITER: return "QLOITER";
            case QLAND: return "QLAND";
            case QRTL: return "QRTL";
            case QAUTOTUNE: return "QAUTOTUNE";
            case QACRO: return "QACRO";
            case THERMAL: return "THERMAL";
            default: return "UNKNOWN MODE";
        }
    }
    std::string getName() const override { return "Plane"; };
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