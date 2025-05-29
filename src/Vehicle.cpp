#include "Vehicle.h"
#include <optional>
std::vector<MissionItem> Vehicle::missionPlan;
std::atomic<bool> Vehicle::armState{false};
std::atomic<bool> Vehicle::missionRunning{false};
std::atomic<bool> Vehicle::missionNeedsStart{true};
std::mutex Vehicle::missionMutex;
std::unordered_map<std::string, Waypoint> waypointMap;
FlightMode Vehicle::autopilotMode{FlightMode::MANUAL};

Vehicle::Vehicle() {
    /*
    pid_t pid = fork();
    if (pid < 0) {
        std::perror("[ERROR] fork failed");
        return;
    }

    if (pid == 0) {
        execl("./reader", "reader", nullptr);
        std::perror("[ERROR] exec failed");
        _exit(1);
    }
    childPid = pid;
    std::cout << "[DEBUG] Spawned helper process with PID: " << childPid << "\n";
    */
    if (!open_ipc()) {
        std::cerr << "Error: failed to open shared memory\n";

    }
    if (!init_cmd_uart()) {
        std::cerr << "Error: failed to open UART\n";
    }

    loadWaypoints();
    startMonitor();
}
Vehicle::~Vehicle() {
    stopMonitor();

    if (childPid > 0) {
        std::cout << "[DEBUG] Killing helper process PID " << childPid << "\n";
        kill(childPid, SIGTERM);  // or SIGKILL if it doesn't shut down gracefully
        waitpid(childPid, nullptr, 0);  // reap zombie
        childPid = -1;
    }
}

void Vehicle::showMissionDetails() {
    std::lock_guard<std::mutex> lock(missionMutex);
    if (missionPlan.empty()) {
        std::cout << "[" << getName() << "] No mission plan loaded.\n";
        return;
    }

    std::cout << "[" << getName() << "] Mission Plan Details:\n";
    for (const auto& item : missionPlan) {
        std::cout << "  - Waypoint " << item.msg.seq
                  << ": Lat " << item.msg.x
                  << ", Lon " << item.msg.y
                  << ", Alt " << item.msg.z
                  << ", Command " << item.msg.command
                  << "\n";
    }
}
bool Vehicle::validateCommand(const Command& cmd) {
    cmd.printArgs();
    const auto& action = cmd.getAction();
    const auto& args   = cmd.getArgs();
    const auto& preArgs = cmd.getPreActionArgs();
   
    //TODO: read autopilot data
    //get telemetry data
    //see if mission is complete(maybe clear the mission if it is complete)
    //see if mission failed to send
    //if mission failed to send, we probably want to try again(we may need a dequeue)


    //Check new command for action word
    if (action == "stop") {
        return validateStop();
    } 
    else if (action == "mission") {
        return validateMission(args, preArgs);
    }
    else if (action == "start") {
        return validateStart();
    } 
    else if (action == "move") {
        return validateMove(args);
    } 
    else if (action == "pan") {
        return validatePan(args);
    } 
    else if (action == "tilt") {
        return validateTilt(args);
    } 
    else if (action == "engine") {
        return validateEngine(args, preArgs);
    } 
    else if (action == "turn") {
        return validateTurn(args);
    } 
    else if (action == "advance") {
        return validateAdvance(args);
    }
    else if (action == "arm") {
        executeArm("arm");
        return true;
    }
    else if (action == "disarm") {
        executeArm("disarm");
        return true;
    }
    else if(action == "vacate") {
        return validateVacate(preArgs, args);
    }
    else if (action == "go") {
        return validateGo(preArgs, args);   
    }
    else {
        std::cout << "[" << getName() << "] Unknown action: " << action << "\n";
        return false;
    }       

    return false;
}

bool Vehicle::validateStop() {
    std::cout << "[" << getName() << "] ACK stop\n";
    executeStop();
    return true;
}

bool Vehicle::validateStart() {
    std::cout << "[" << getName() << "] ACK start\n";
    executeStart();
    return true;
}


bool Vehicle::validateMove(const std::vector<std::string>& args) {
    const int maxLookahead = 3;
    std::vector<Waypoint> waypoints;

    for (size_t i = 0; i < args.size(); ++i) {
        std::string token = args[i];
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);

        if (token == "waypoint") {
            // Look ahead for the actual waypoint name
            for (size_t j = i + 1; j <= i + maxLookahead && j < args.size(); ++j) {
                std::string candidate = args[j];

                double lat, lon, alt;
                if (getWaypointCoords(candidate, &lat, &lon, &alt)) {
                    Waypoint wp = { lat, lon, alt };
                    waypoints.push_back(wp);
                    i = j; // Skip past matched waypoint name
                    break;
                }
            }
        }
    }

    if (waypoints.empty()) {
        std::cout << "Error: No valid waypoints found after 'move' command.\n";
        return false;
    }

    executeMove(waypoints);
    return true;
}


bool Vehicle::validatePan(const std::vector<std::string>& args) {
    float deg = 0.0f;
    bool found = false;
    for (const auto& token : args) {
        try { deg = std::stof(token); found = true; break; }
        catch(...) { continue; }
    }
    if (!found) {
        std::cout << "["<< getName() <<"] Pan command needs a numeric angle\n";
        return false;
    }
    if (deg < 0.0f || deg > 180.0f) {
        std::cout << "["<< getName() <<"] Pan angle out of range (0-180)\n";
        return false;
    }
    std::cout << "["<< getName() <<"] ACK pan "<< deg <<" degrees\n";
    executePan(deg);
    return true;
}

bool Vehicle::validateTilt(const std::vector<std::string>& args) {
    float deg = 0.0f;
    bool found = false;
    for (const auto& token : args) {
        try { deg = std::stof(token); found = true; break; }
        catch(...) { continue; }
    }
    if (!found) {
        std::cout << "["<< getName() <<"] Tilt command needs a numeric angle\n";
        return false;
    }
    if (deg < 0.0f || deg > 180.0f) {
        std::cout << "["<< getName() <<"] Tilt angle out of range (0-180)\n";
        return false;
    }
    std::cout << "["<< getName() <<"] ACK tilt "<< deg <<" degrees\n";
    executeTilt(deg);
    return true;
}

bool Vehicle::validateEngine(const std::vector<std::string>& args, const std::vector<std::string>& preArgs) {
    const std::unordered_map<std::string, std::string> engineCommands = {
            {"start", "start"},
            {"stop", "stop"},
            {"on", "start"},
            {"off", "stop"},
            {"ignite", "start"},
            {"shutdown", "stop"}
        };

    for (const std::string& raw : preArgs) {
        std::string token = raw;
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);
        auto it = engineCommands.find(token);
        if (it != engineCommands.end()) {
            std::string selectedCommand = it->second;
            std::cout << "[Engine] Command detected: " << selectedCommand << "\n";
            executeEngine(selectedCommand);
            return true;
        }
    }
    for (const std::string& raw : args) {
        std::string token = raw;
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);
        auto it = engineCommands.find(token);
        if (it != engineCommands.end()) {
            std::string selectedCommand = it->second;
            std::cout << "[Engine] Command detected: " << selectedCommand << "\n";
            executeEngine(selectedCommand);
            return true;
        }
    }

    std::cout << "Error: No valid engine command found (start/stop).\n";
    return false;
    
}

bool Vehicle::validateTurn(const std::vector<std::string>& args) {
    std::cout << "[" << getName() << "] ACK turn\n";
    float yawDeg    = -1.0f;  // sentinel: “no angle yet”
    int   direction =  1;     // default → right/CW
    bool  relative  = false;  // default → absolute (we’ll treat as right)

    // matches integers or floats
    static const std::regex numRx(R"(([0-9]+(?:\.[0-9]+)?))");
    std::smatch                m;

    for (const auto& tok : args) {
        std::string s = tok;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);

        if (s == "left") {
            direction = -1;
            relative  = true;
        }
        else if (s == "right") {
            direction =  1;
            relative  = true;
        }
        else if (std::regex_search(s, m, numRx) && yawDeg < 0.0f) {
            try {
                yawDeg = std::stof(m.str(1));
            } catch (...) {
                
            }
        }
    }

    if (yawDeg < 0.0f) {
        std::cout << "Error: no valid turn angle found in input\n";
        return false;
    }

  
    executeTurn(yawDeg, direction, relative);
    return true;
}

bool Vehicle::validateAdvance(const std::vector<std::string>& args) {
    static const std::regex numRx(R"((-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?))");
    std::smatch m;

    double distance_m = 0.0;
    bool hasDistance = false;

    double bearing_deg = 0.0;
    bool hasBearing = false;

    std::string wptName;
    bool useNamedWpt = false;

    const int maxLookahead = 3;

    for (size_t i = 0; i < args.size(); ++i) {
        std::string token = args[i];
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);

        // Match numbers and look ahead for a unit
        if (std::regex_match(token, m, numRx)) {
            double val = 0.0;
            try {
                val = std::stod(m.str(1));
            } catch (...) {
                continue;
            }

            for (int j = 1; j <= maxLookahead && (i + j) < args.size(); ++j) {
                std::string unit = args[i + j];
                std::transform(unit.begin(), unit.end(), unit.begin(), ::tolower);

                if (!hasDistance && (
                        unit == "meter" || unit == "meters" ||
                        unit == "kilometer" || unit == "kilometers" ||
                        unit == "mile" || unit == "miles" ||
                        unit == "foot" || unit == "feet")) {
                    if (unit == "meter" || unit == "meters") distance_m = val;
                    else if (unit == "kilometer" || unit == "kilometers") distance_m = val * 1000.0;
                    else if (unit == "mile" || unit == "miles") distance_m = val * 1609.34;
                    else if (unit == "foot" || unit == "feet") distance_m = val * 0.3048;
                    hasDistance = true;
                    break;
                }

                if (!hasBearing && (
                        unit == "degree" || unit == "degrees" ||
                        unit == "radian" || unit == "radians")) {
                    if (unit == "degree" || unit == "degrees") bearing_deg = val;
                    else if (unit == "radian" || unit == "radians") bearing_deg = val * (180.0 / M_PI);
                    hasBearing = true;
                    break;
                }
            }
        }

        // After "waypoint", scan forward to find first valid named waypoint
        if (token == "waypoint") {
            useNamedWpt = true;
            for (size_t j = i + 1; j <= i + maxLookahead && j < args.size(); ++j) {
                std::string candidate = args[j];
                if (getWaypointCoords(candidate, nullptr, nullptr, nullptr)) {
                    wptName = candidate;
                    break;
                }
            }
        }
    }

    if (!hasDistance) {
        std::cout << "Error: No valid distance with units like meters, kilometers, feet, or miles.\n";
        return false;
    }

    if (!hasBearing) {
    getBearing(&bearing_deg);
    std::cout << "Debug: No bearing specified; using forward bearing from current heading (" 
              << bearing_deg << " deg).\n";
    }

    double curLat, curLon, curAlt;
    if (useNamedWpt) {
        if (!getWaypointCoords(wptName, &curLat, &curLon, &curAlt)) {
            std::cout << "Error: Waypoint '" << wptName << "' not found.\n";
            return false;
        }
    } else {
        getTelem(&curLat, &curLon, &curAlt);
    }

    double destLat, destLon;
    vincentyDirect(curLat, curLon, bearing_deg, distance_m, destLat, destLon);

    executeAdvance(destLat, destLon, 0.0f);
    return true;
}

void Vehicle::startMonitor() {
    monitorRunning = true;
    monitorThread = std::thread(&Vehicle::monitorLoop, this);
}

void Vehicle::stopMonitor() {
    monitorRunning = false;
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

void Vehicle::monitorLoop() {
    while (monitorRunning) {
        bool shouldStart = false;
        {
            std::lock_guard<std::mutex> lock(Vehicle::missionMutex);

            std::cout << "[" << getName() << "] Vehicle state "
                      << "Engine: " << engineOn
                      << ", missionRunning: " << missionRunning
                      << ", missionNeedsStart: " << missionNeedsStart
                      << ", mission size: " << missionPlan.size() 
                      << ", autopilot state: " << static_cast<int>(autopilotMode) 
                      << ", armState: " << armState
                      << std::endl;

            if (!missionRunning && missionNeedsStart && engineOn && !missionPlan.empty()) {
                missionRunning = true;
                missionNeedsStart = false;
                shouldStart = true;
            }
            else if (missionRunning && engineOn) {
                pthread_mutex_lock(&shm_ptr->mutex);
                if (shm_ptr->current_seq >= missionPlan.size()) {
                    std::cout << "[" << getName() << "] Mission completed.\n";
                    missionRunning = false;
                    missionNeedsStart = true;
                    shm_ptr->missionState = MissionExecState::COMPLETED;
                }
                pthread_mutex_unlock(&shm_ptr->mutex);
            }
            else if (engineOn) {
                missionNeedsStart = true;
            }
            else if (!engineOn && missionRunning) {
                std::cout << "[" << getName() << "] Mission is running, but engine is off. Stopping mission.\n";
                //executeStop();
                missionRunning = false;
            }
            else {
                missionNeedsStart = true;
            }
        }

        if (shouldStart) {
            std::cout << "[" << getName() << "] Starting mission...\n";
            {
                std::lock_guard<std::mutex> lock(Vehicle::missionMutex);  // re-lock just to read a safe snapshot
                startMissionUpload(missionPlan);  // safe to pass by const-ref
            }
            setMissionCurrent(0);
            //arm
            setFlightMode(FlightMode::AUTO);
            std::cout << "[" << getName() << "] Mission started.\n";
        }

        std::this_thread::sleep_for(std::chrono::seconds(4));
    }
}

bool Vehicle::validateVacate(const std::vector<std::string>& preArgs, const std::vector<std::string>& args) {
    std::cout << "[" << getName() << "] ACK vacate\n";
    return true;
}

bool Vehicle::validateGo(const std::vector<std::string>& preArgs, const std::vector<std::string>& args) {
    std::cout << "[" << getName() << "] ACK go\n";
    return true;
}

void Vehicle::loadWaypoints() {
    std::ifstream file(WAYPOINTS_FILE);
    if (!file.is_open()) {
        std::cerr << "Error: could not open waypoints.csv " << "\n";
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string name;
        double lat, lon, alt;
        if (std::getline(iss, name, ',') &&
            iss >> lat && iss.get() &&
            iss >> lon && iss.get() &&
            iss >> alt) {
            waypointMap[name] = {lat, lon, alt};
            std::cout << "[Rover] Loaded waypoint: " << name 
                      << " (" << lat << ", " << lon << ", " << alt << ")\n";
        }
    }
}

bool Vehicle::getWaypointCoords(const std::string& name, double* lat, double* lon, double* alt) {
    auto it = waypointMap.find(name);
    if (it == waypointMap.end()) {
        return false;
    }
    if (lat) *lat = it->second.lat;
    if (lon) *lon = it->second.lon;
    if (alt) *alt = it->second.alt;
    return true;
}

bool Vehicle::validateMission(const std::vector<std::string>& args, const std::vector<std::string>& preArgs) {
   const std::unordered_map<std::string, std::string> missionCommands = {
        {"start", "start"},
        {"begin", "start"},
        {"initiate", "start"},
        {"stop", "stop"},
        {"end", "stop"},
        {"halt", "stop"},
        {"terminate", "stop"},
        {"resume", "resume"},
        {"continue", "resume"},
        {"abort", "abort"},
        {"cancel", "abort"},
        {"details", "details"},
        {"info", "details"},
        {"show", "details"}
    };

    for (const std::string& raw : preArgs) {
        std::string token = raw;
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);

        auto it = missionCommands.find(token);
        if (it != missionCommands.end()) {
            std::string selectedCommand = it->second;
            std::cout << "[Mission] Command detected: " << selectedCommand << "\n";
            executeMission(selectedCommand);
            return true;
        }
    }

    std::cout << "Error: No valid mission command found (start/stop/resume/abort).\n";
    return false;

    
}