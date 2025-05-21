#pragma once
#include <string>
#include <memory>
#include "Command.h"
#include <set>
#include "mavlink_usart.h"
#include "reader.h"
#include <cmath>
#include <iostream>
#include <regex>
#include "helper.h"
/*
Vehicle is the base class for all vehicles. It uses virutual functions to allow for run-time declarition of vehicle type when used in the parser.
It provides a semi-abstract validate functions that most vehicles can just use as is, but allows for vehicles to override them if they need to.

All vehicles must implement their own execute functions. These execute functions should be able to handle differences in vehicle type.
*/
class Vehicle {
    public:
        virtual ~Vehicle();          
        virtual std::string getName() const = 0;
    
        /*
        This is the main control loop for the vehicle. It takes in the command interface between the parser and vehicle.
        It will call the appropriate validate function based on the command type. If the command is valid, it will call the execute function.
        If the command is not valid, it will print an error message.

        It will also be used to check vehicle states. Like mission running, mission needs start, etc.
        */
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