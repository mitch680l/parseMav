#include "Vehicle.h"
#include <sys/stat.h> 
#include <optional>
std::unordered_map<std::string, Waypoint> waypointMap;
std::vector<MissionItem> Vehicle::missionPlan;
std::mutex Vehicle::missionMutex;
SharedTelem Vehicle::snapshot;


Vehicle::Vehicle() {
     
    if (!init_cmd_uart()) {
        std::cerr << "Error: failed to open UART\n";
    }

    if (LAUNCH_READER) {
    #if SIM_MODE
    int sockpair[2];
    socketpair(AF_UNIX, SOCK_DGRAM, 0, sockpair);
    

    
    pid_t pid = fork();
    if (pid < 0) {
        std::perror("[ERROR] fork failed");
        return;
    }

    if (pid == 0) {
        dup2(sockpair[1], 3);
        execl("./reader", "reader", nullptr);
        std::perror("[ERROR] exec failed");
        _exit(1);
    }
    close(sockpair[1]);  // Close child's end
    send_fd(sockpair[0], uart_fd);  // Pass the TCP socket
    close(sockpair[0]);  // Done
    #else
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
    #endif
    childPid = pid;
    std::cout << "[DEBUG] Spawned helper process with PID: " << childPid << "\n";
    }

    if (!create_ipc()) {
        std::cerr << "Error: failed to open shared memory\n";

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
    else if (action == "mode") {
        return validateMode(preArgs, args);
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
    else if (action == "testuart") {
        if (cmd_fd >= 0) {
            close(cmd_fd);
            cmd_fd = -1;
            std::cout << "[TEST] UART forcibly closed\n";
        }
    return true;
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
    if (args.empty()) {
        std::cout << "[" << getName() << "] Pan command requires direction, angle, and unit\n";
        return false;
    }

    std::string direction;
    float value = 0.0f;
    std::string unit;

    bool direction_found = false;
    bool value_found = false;
    bool unit_found = false;

    // STEP 1: Find direction anywhere
    for (const auto& token : args) {
        std::string lower = token;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        if (lower == "left" || lower == "right") {
            direction = lower;
            direction_found = true;
            break;
        }
    }

    if (!direction_found) {
        std::cout << "[" << getName() << "] Pan command requires direction (left/right)\n";
        return false;
    }

    // STEP 2: Find number followed within 3 tokens by a valid unit
    for (int i = 0; i < (int)args.size(); ++i) {
        try {
            float test_value = std::stof(args[i]);
            // Search next 3 tokens for a valid unit
            for (int j = i + 1, count = 0; j < (int)args.size() && count < 3; ++j, ++count) {
                std::string unit_token = args[j];
                std::transform(unit_token.begin(), unit_token.end(), unit_token.begin(), ::tolower);
                if (unit_token == "deg" || unit_token == "degrees" ||
                    unit_token == "rad" || unit_token == "radians" ||
                    unit_token == "mil" || unit_token == "mils") {
                    value = test_value;
                    unit = unit_token;
                    value_found = unit_found = true;
                    break;
                }
            }

            if (value_found && unit_found) break;
        } catch (...) {
            continue;
        }
    }

    if (!value_found) {
        std::cout << "[" << getName() << "] Pan command requires a numeric angle\n";
        return false;
    }

    if (!unit_found) {
        std::cout << "[" << getName() << "] Pan command requires a unit (degrees, radians, mils) within 3 words after the angle\n";
        return false;
    }

    if (value <= 0.0f) {
        std::cout << "[" << getName() << "] Pan angle must be positive\n";
        return false;
    }

    // Normalize to degrees
    std::transform(unit.begin(), unit.end(), unit.begin(), ::tolower);
    float degrees = value;
    if (unit == "rad" || unit == "radians") {
        degrees = value * (180.0f / M_PI);
    } else if (unit == "mil" || unit == "mils") {
        degrees = value * (360.0f / 6400.0f);
    }

    constexpr float MIN_PAN = 0.0f;
    constexpr float MAX_PAN = 180.0f;
    float current_pos = Vehicle::pan_position;
    float new_position = current_pos;

    if (direction == "right") {
        new_position += degrees;
    } else if (direction == "left") {
        new_position -= degrees;
    }

    if (new_position < MIN_PAN || new_position > MAX_PAN) {
        std::cout << "[" << getName() << "] Pan command would move servo out of range (0-180 degrees)\n";
        new_position = std::clamp(new_position, MIN_PAN, MAX_PAN);
    }

    std::cout << "[" << getName() << "] ACK pan " << direction << " " << value << " " << unit
              << " (moving from " << current_pos << "° to " << new_position << "°)\n";

    Vehicle::pan_position = new_position;
    executePan(new_position);
    return true;
}

bool Vehicle::validateTilt(const std::vector<std::string>& args) {
    if (args.empty()) {
        std::cout << "[" << getName() << "] Tilt command requires direction, angle, and unit\n";
        return false;
    }

    std::string direction;
    float value = 0.0f;
    std::string unit;

    bool direction_found = false;
    bool value_found = false;
    bool unit_found = false;

    // STEP 1: Search for direction anywhere
    for (const auto& token : args) {
        std::string lower = token;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        if (lower == "up" || lower == "down" ||
            lower == "clockwise" || lower == "counterclockwise") {
            direction = lower;
            direction_found = true;
            break;
        }
    }

    if (!direction_found) {
        std::cout << "[" << getName() << "] Tilt command requires direction (up/down/etc)\n";
        return false;
    }

    // STEP 2: Search for a number and a valid unit within 3 tokens after it
    for (int i = 0; i < (int)args.size(); ++i) {
        try {
            float test_value = std::stof(args[i]);
            // Check the next 3 tokens for a valid unit
            for (int j = i + 1, count = 0; j < (int)args.size() && count < 3; ++j, ++count) {
                std::string unit_token = args[j];
                std::transform(unit_token.begin(), unit_token.end(), unit_token.begin(), ::tolower);
                if (unit_token == "deg" || unit_token == "degrees" ||
                    unit_token == "rad" || unit_token == "radians" ||
                    unit_token == "mil" || unit_token == "mils") {
                    // Success
                    value = test_value;
                    unit = unit_token;
                    value_found = unit_found = true;
                    break;
                }
            }

            if (value_found && unit_found) break; // chain found, exit outer loop
        } catch (...) {
            continue;
        }
    }

    if (!value_found) {
        std::cout << "[" << getName() << "] Tilt command requires a numeric angle\n";
        return false;
    }

    if (!unit_found) {
        std::cout << "[" << getName() << "] Tilt command requires a unit (degrees, radians, mils) within 3 words after the angle\n";
        return false;
    }

    if (value <= 0.0f) {
        std::cout << "[" << getName() << "] Tilt angle must be positive\n";
        return false;
    }

    // Normalize to degrees
    std::transform(unit.begin(), unit.end(), unit.begin(), ::tolower);
    float degrees = value;
    if (unit == "rad" || unit == "radians") {
        degrees = value * (180.0f / M_PI);
    } else if (unit == "mil" || unit == "mils") {
        degrees = value * (360.0f / 6400.0f);
    }

    constexpr float MIN_TILT = 0.0f;
    constexpr float MAX_TILT = 180.0f;
    float current_pos = Vehicle::tilt_position;
    float new_position = current_pos;

    if (direction == "up" || direction == "clockwise") {
        new_position += degrees;
    } else if (direction == "down" || direction == "counterclockwise") {
        new_position -= degrees;
    }

    if (new_position < MIN_TILT || new_position > MAX_TILT) {
        std::cout << "[" << getName() << "] Tilt command would move servo out of range (0-180 degrees)\n";
        new_position = std::clamp(new_position, MIN_TILT, MAX_TILT);
    }

    std::cout << "[" << getName() << "] ACK tilt " << direction << " " << value << " " << unit
              << " (moving from " << current_pos << "° to " << new_position << "°)\n";

    Vehicle::tilt_position = new_position;
    executeTilt(new_position);
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
    SendServo(PAN_CHANNEL,SERVO_PAN_START);
    SendServo(TILT_CHANNEL,SERVO_TILT_START);
    flags.monitorRunning = true;
    monitorThread = std::thread(&Vehicle::monitorLoop, this);
}

void Vehicle::stopMonitor() {
    flags.monitorRunning = false;
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

void Vehicle::monitorLoop() {
    
    while (flags.monitorRunning) {
        SendHeartbeat();
        
        
        pthread_mutex_lock(&shm_ptr->mutex);
        snapshot = *shm_ptr;
        pthread_mutex_unlock(&shm_ptr->mutex);

        const int MAX_RETRIES = 3;

        if (!checkUartHealth()) {
            health.uartFailCount++;
            if (health.uartFailCount <= MAX_RETRIES && attemptReinitUart()) {
                std::cout << "[" << getName() << "] UART reinitialized successfully.\n";
                health.uartAlive = true;
            } else {
                std::cout << "[" << getName() << "] UART health check failed, reinitialization failed.\n";
                health.uartAlive = false;
            }
        } else {
            health.uartFailCount = 0;
            health.uartAlive = true;
        }

        if (!checkIpcPointer()) {
            health.ipcFailCount++;
            if (health.ipcFailCount <= MAX_RETRIES && reopenIpc()) {
                health.ipcAlive = true;
            } else {
                health.ipcAlive = false;
            }
        } else {
            health.ipcFailCount = 0;
            health.ipcAlive = true;
        }

        if (!checkReaderProcess()) {
            health.readerFailCount++;
            if (health.readerFailCount <= MAX_RETRIES && restartReaderProcess()) {
                health.readerAlive = true;
            } else {
                health.readerAlive = false;
            }
        } else {
            health.readerFailCount = 0;
            health.readerAlive = true;
        }
        
        {
            std::lock_guard<std::mutex> lock(Vehicle::missionMutex);
            
            if(snapshot.requestParams) {
                respondToParamRequest();
            }


            /*
            This section should check to see that saftey systems are running IT SHOULD NOT be the initiator of those systems.
            This loop runs at 0.25 Hz(currently) which is not a good enough response time for saftey systems.
            */
            //engine off brake engaged
            if (!flags.engine || flags.brake) {
                //ensure vehicle is not running(CRITICAL)
                flags.safeMove = false;
            }
            else {
                flags.safeMove = true;
            }

            /*
            if auto load is on, we will automatically load the mission queue onto autopilot
            1.) Mission is not currently running, or paused on autopilot
            2.) There are new items on the mission queue(i.e. not empty, not completed items)
            */
            #if AUTO_LOAD_MISSION
            //Check the conditions
            //Allow load to happen if conditions are set
            #endif

            /*
            if auto start is on we will automatically start given these condtions
            1.) There is a mission loaded on the autopilot
            2.) It is safe to move
            */
            #if AUTO_START_MISSION
            
            #endif


            if (flags.shouldLoad) {
                
                if (missionPlan.size() <= 0 || getMissionSize(missionPlan) <= static_cast<int>(snapshot.missionCountAutopilot)) {
                
                    std::cout << "[Monitor] Mission Upload Rejected(Mission Already Running or No new items in Plan)" << std::endl;
                }
                else if(snapshot.mission_state == MISSION_ACTIVE || snapshot.mission_state == MISSION_PAUSED || snapshot.mission_state == MISSION_NOT_STARTED) {
                    //if mission is loaded to autopilot and not complete, we would prefer to just append to the mission as opposed to rewriting
                    startPartialMissionUpload(missionPlan, static_cast<int>(snapshot.missionCountAutopilot));
                }
                else if (snapshot.mission_state == MISSION_COMPLETED || snapshot.mission_state == MISSION_NO_MISSION || snapshot.mission_state == MISSION_UNKNOWN) {
                    
                    startFullMissionUpload(missionPlan, snapshot.current_seq);
                    
                }
                else {
                    std::cout << "Unsported Case in monitor \"shouldLoad\" functionality" << std::endl;            
                }
                flags.shouldLoad = false;
            }

            if (flags.shouldStart) {
                
                if (flags.safeMove) {
                sendMissionStartCommand();
                }
                else {
                    std::cout << "[" << getName() << "] " << " Is ready to start mission, but is unsafe" << std::endl;
                }
                flags.shouldStart = false;
            }

        #if DEBUG_MONITOR_STATE 
            std::cout << "[" << getName() << "] MISSION STATE: "
                      << "Safe: " <<   flags.safeMove
                      << ", Size: " << missionPlan.size() 
                      << ", Autopilot Size: " << static_cast<int>(snapshot.missionCountAutopilot)
                      << ", State: " << getMissionStateName(static_cast<int>(snapshot.mission_state))
                      << ", current mission: " << static_cast<int>(snapshot.current_seq)
                      << std::endl;

                      std::cout << "[" << getName() << "]" << " AUTOPILOT STATUS: " << getModeName(snapshot.mode)
                      << " " << (snapshot.armed ? "ARMED" : "DISARMED")
                      << std::endl;
            
            #endif

        #if DEBUG_SHARED_TELEM
        std::cout << "[" << getName() << "] Shared Telemetry State: "
                << "Lat: " << snapshot.lat
                << ", Lon: " << snapshot.lon
                << ", Alt: " << snapshot.alt
                << ", Yaw: " << snapshot.yaw_deg
                << std::endl;
        #endif
        
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
        {"pause", "pause"},
        {"abort", "abort"},
        {"cancel", "abort"},
        {"details", "details"},
        {"info", "details"},
        {"show", "details"},
        {"plan", "details"},
        {"list", "details"},
        {"clear", "clear"},
        {"reset", "clear"},
        {"upload", "upload"},
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

bool Vehicle::checkUartHealth() {
    if (cmd_fd < 0) return false;

    uint8_t testByte = 0x00;
    ssize_t result = write(cmd_fd, &testByte, 1);
    return result > 0;
}

bool Vehicle::attemptReinitUart() {
    if (cmd_fd >= 0) {
        close(cmd_fd);
        cmd_fd = -1;
    }

    std::cout << "[Monitor] Attempting UART reinitialization...\n";
    return init_cmd_uart();  // reuse your existing init function
}

bool Vehicle::checkIpcPointer() {
    if (!shm_ptr) return false;

    if (pthread_mutex_trylock(&shm_ptr->mutex) == 0) {
        pthread_mutex_unlock(&shm_ptr->mutex);
        return true;
    } else {
        std::cerr << "[Monitor] Shared memory mutex seems locked or invalid\n";
        return false;
    }
}

bool Vehicle::reopenIpc() {
    std::cout << "[Monitor] Attempting to reopen shared memory...\n";
    return open_ipc();  // reuse your existing init function
}

bool Vehicle::checkReaderProcess() {
    if (childPid <= 0) return false;

    int status;
    pid_t result = waitpid(childPid, &status, WNOHANG);

    if (result == 0) return true;  // still running
    if (result == childPid) {
        std::cerr << "[Monitor] Reader exited unexpectedly with status " << status << "\n";
        childPid = -1;
        return false;
    }

    return false;
}

bool Vehicle::restartReaderProcess() {
    std::cout << "[Monitor] Restarting reader process...\n";

    pid_t pid = fork();
    if (pid < 0) {
        std::perror("[ERROR] fork failed");
        return false;
    }

    if (pid == 0) {
        execl("./reader", "reader", nullptr);
        std::perror("[ERROR] exec failed");
        _exit(1);
    }

    childPid = pid;
    std::cout << "[DEBUG] Reader restarted with PID: " << childPid << "\n";
    return true;
}

bool Vehicle::validateMode(const std::vector<std::string>& preArgs, const std::vector<std::string>& args) {
    if (args.empty()) {
        std::cout << "[" << getName() << "] Mode command requires a mode name\n";
        return false;
    }

    std::string mode = args[0];
    std::string type = args[1];

    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    if (mode == "auto" || mode == "manual" || mode == "guided" || mode == "rtl") {
        executeMode(mode, type);
        return true;
    } else {
        std::cout << "[" << getName() << "] Unknown mode: " << mode << "\n";
        return false;
    }
}