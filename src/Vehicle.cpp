#include "Vehicle.h"
Vehicle::~Vehicle() = default;

bool Vehicle::missionRunning     = false;
bool Vehicle::missionNeedsStart  = false;

bool Vehicle::validateCommand(const Command& cmd) {
    cmd.printArgs();
    const auto& action = cmd.getAction();
    const auto& args   = cmd.getArgs();
   
    //TODO: read autopilot data
    //get telemetry data
    //see if mission is complete(maybe clear the mission if it is complete)
    //see if mission failed to send
    //if mission failed to send, we probably want to try again(we may need a dequeue)


    //Check new command for action word
    if (action == "stop") {
        return validateStop();
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
        return validateEngine(args);
    } 
    else if (action == "turn") {
        return validateTurn(args);
    } 
    else if (action == "advance") {
        return validateAdvance(args);
    } 
    else {
        std::cout << "[" << getName() << "] Unknown action: " << action << "\n";
        return false;
    }       

    //Check if action triggered a mission protocol
    if (!missionRunning && missionNeedsStart) {
        std::cout << "[" << getName() << "] Mission Detected....Starting\n";
        executeStart();
        missionRunning = true;
        missionNeedsStart = false;
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
    static const std::set<std::string> validDirs = {"forward","backward","left","right"};
    std::string chosen;
    for (const auto& token : args) {
        if (validDirs.count(token)) chosen = token;
    }
    if (chosen.empty()) {
        std::cout << "["<< getName() <<"] Invalid move args\n";
        return false;
    }
    std::cout << "["<< getName() <<"] ACK move "<< chosen <<"\n";
    executeMove(chosen);
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

bool Vehicle::validateEngine(const std::vector<std::string>& args) {
    return true;
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

    // 1) scan tokens for numbers AND a named waypoint
    static const std::regex numRx(R"(([0-9]+(?:\.[0-9]+)?))");
    std::smatch                m;
    std::vector<double>        nums;
    std::string                wptName;
    bool                       useNamedWpt = false;

    for (size_t i = 0; i < args.size(); ++i) {
        std::string s = args[i];
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);

        if (s == "waypoint" && i + 1 < args.size()) {
            wptName      = args[i + 1];
            useNamedWpt  = true;
        }
        
        if (std::regex_search(s, m, numRx)) {
            try {
                nums.push_back(std::stod(m.str(1)));
            } catch (...) {
    
            }
        }
    }
    if (nums.empty()) {
        std::cout << "Error: no distance specified for advance\n";
        return false;
    }
    double distance_m = nums[0];

    double bearing_deg;
    if (nums.size() >= 2) {
        bearing_deg = nums[1];
    } else {
        getBearing(&bearing_deg);
    }

    double curLat, curLon, curAlt;
    if (useNamedWpt) {
        if (!getWaypointCoords(wptName, &curLat, &curLon, &curAlt)) {
            std::cout << "Error: waypoint '" << wptName << "' not found\n";
            return false;
        }
    } else {
        getTelem(&curLat, &curLon, &curAlt);
    }

    double destLat, destLon;
    vincentyDirect(curLat, curLon,
                   bearing_deg,
                   distance_m,
                   destLat, destLon);

    executeAdvance(destLat, destLon, 0.0f);
    return true;
}