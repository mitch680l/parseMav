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
#include <thread>
#include <atomic>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <mutex>
#include <unordered_map>
/*
Vehicle is the base class for all vehicles. It uses virutual functions to allow for run-time declarition of vehicle type when used in the parser.
It provides a semi-abstract validate functions that most vehicles can just use as is, but allows for vehicles to override them if they need to.

All vehicles must implement their own execute functions. These execute functions should be able to handle differences in vehicle type.
*/
class Vehicle {
    public:
        Vehicle(); 
        virtual ~Vehicle();          
        virtual std::string getName() const = 0;
        virtual std::string getModeName(uint32_t custom_mode) const = 0;
    
        /*
        This is the main control loop for the vehicle. It takes in the command interface between the parser and vehicle.
        It will call the appropriate validate function based on the command type. If the command is valid, it will call the execute function.
        If the command is not valid, it will print an error message.

        It will also be used to check vehicle states. Like mission running, mission needs start, etc.
        */
        virtual bool validateCommand(const Command& cmd);
        void startMonitor();
        void stopMonitor();
        void loadWaypoints();
        bool getWaypointCoords(const std::string& name, double* lat, double* lng, double* alt);
        bool checkUartHealth();
        bool attemptReinitUart();
        bool reopenIpc();
        bool checkIpcPointer();
        bool checkReaderProcess();
        bool restartReaderProcess();

        virtual void showMissionDetails();
        virtual bool validateStop();
        virtual bool validateStart();
        virtual bool validateMission(const std::vector<std::string>& args, const std::vector<std::string>& preArgs);
        virtual bool validateMove(const std::vector<std::string>& args);
        virtual bool validatePan(const std::vector<std::string>& args);
        virtual bool validateTilt(const std::vector<std::string>& args);
        virtual bool validateEngine(const std::vector<std::string>& args, const std::vector<std::string>& preArgs);
        virtual bool validateTurn(const std::vector<std::string>& args);
        virtual bool validateAdvance(const std::vector<std::string>& args);
        virtual bool validateGo(const std::vector<std::string>& preArgs, const std::vector<std::string>& args);
        virtual bool validateVacate(const std::vector<std::string>& preArgs, const std::vector<std::string>& args);
        virtual bool validateMode(const std::vector<std::string>& preArgs, const std::vector<std::string>& args);
        

        virtual void executeTurn(float yaw, int direction, bool relative) = 0;
        virtual void executeAdvance(float lat, float lng, float alt) = 0;
        virtual void executeStop() = 0;
        virtual void executeStart() = 0;
        virtual void executeMove(std::vector<Waypoint> waypoints) = 0;
        virtual void executePan(float deg) = 0;
        virtual void executeTilt(float deg) = 0;
        virtual void executeMission(std::string missionCommand) = 0;
        virtual void executeEngine(const std::string& command) = 0;
        virtual void executeArm(std::string command) = 0;
        virtual void executeMode(const std::string& mode, const std::string& type) = 0;


    protected:
        virtual void monitorLoop();
        std::unordered_map<std::string, Waypoint> waypointMap; //cahce of waypoints
        static std::atomic<bool> armState; //arm/disarmed
        static std::vector<MissionItem> missionPlan; //mission plan vector
        static std::atomic<bool> missionRunning; //is mission running?
        static std::atomic<bool> missionNeedsStart; //does mission need to be started?(true when mission plan is loaded, but not started yet)
        static std::atomic<bool> missionPaused;
        static std::atomic<bool> monitorRunning;
        static std::atomic<bool> engine;
        static std::atomic<bool> brake;
        pid_t childPid = -1; //TO-DO: implment reader as child process
        std::thread monitorThread; // thread for handling vehicle state
        
        static std::mutex missionMutex; // mutex for mission plan access
        static int pan_position;
        static int tilt_position;
        HealthStatus health;
    };