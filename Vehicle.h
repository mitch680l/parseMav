#pragma once
#include <string>
#include <memory>
#include "Command.h"
#include <set>
#include "mavlink_usart.h"
#include <cmath>
#include <iostream>
#include <regex>
#include "helper.h"

class Vehicle {
    public:
        virtual ~Vehicle();          
        virtual std::string getName() const = 0;
    
        virtual bool validateCommand(const Command& cmd);
    
        virtual bool validateStop();
        virtual bool validateStart();
        virtual bool validateMove(const std::vector<std::string>& args);
        virtual bool validatePan(const std::vector<std::string>& args);
        virtual bool validateTilt(const std::vector<std::string>& args);
        virtual bool validateEngine(const std::vector<std::string>& args);
        virtual bool validateTurn(const std::vector<std::string>& args);
        virtual bool validateAdvance(const std::vector<std::string>& args);

        virtual void executeTurn(float yaw, int direction, bool relative) = 0;
        virtual void executeAdvance(float lat, float lng, float alt) = 0;
        virtual void executeStop() = 0;
        virtual void executeStart() = 0;
        virtual void executeMove(const std::string& dir) = 0;
        virtual void executePan(float deg) = 0;
        virtual void executeTilt(float deg) = 0;
        static bool missionRunning;
        static bool missionNeedsStart;
    };