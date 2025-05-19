#pragma once
#include <map>
#include <string>
#include <memory>
#include <set>
#include <vector>
#include <deque>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <chrono>
#include <algorithm>
#include <cctype>

#include "Vehicle.h"
#include "Rover.h"
#include "Plane.h"
#include "Copter.h"
#include "Command.h"

class SpeechInterpreter {
public:
    /**
     * @param path
     * @param actionsFile  
     */
    explicit SpeechInterpreter(const std::string& path,
                               const std::string& actionsFile = "actions.cfg")
        : fifoPath(path) {
        vehicles["rover"]  = std::make_unique<Rover>();
        vehicles["plane"]  = std::make_unique<Plane>();
        vehicles["copter"] = std::make_unique<Copter>();
        loadActionsFromFile(actionsFile);
    }

    ~SpeechInterpreter() {
        if (fifoFd >= 0) close(fifoFd);
    }

    void run() {
        prepareFifo();
        while (running) {
            std::string token = readToken();
            if (!token.empty()) {
                dispatchToken(token);
            }
            checkTimeout();
            usleep(100000);  // Sleep 100ms to reduce busy-waiting
        }
    }

private:
    std::map<std::string, std::unique_ptr<Vehicle>> vehicles;
    std::set<std::string> validActions;
    std::string fifoPath;
    int fifoFd = -1;
    bool running = true;
    std::string readBuffer;
    std::deque<std::string> tokenQueue;

    std::string currentVehicle;
    bool inAction = false;
    std::string currentAction;
    std::vector<std::string> actionArgs;
    std::chrono::steady_clock::time_point actionStartTime;
    const std::string endKeyword = "over";

    void loadActionsFromFile(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            std::string act = normalize(line);
            if (!act.empty()) validActions.insert(act);
        }
    }

    void prepareFifo() {
        if (mkfifo(fifoPath.c_str(), 0666) < 0 && errno != EEXIST) {
            std::cerr << "[Interpreter] mkfifo failed: " << strerror(errno) << "\n";
            return;
        }
        fifoFd = open(fifoPath.c_str(), O_RDONLY | O_NONBLOCK);
        if (fifoFd < 0) {
            std::cerr << "[Interpreter] open fifo failed: " << strerror(errno) << "\n";
        }
    }

    // Reads and returns the next available token (normalized word) or empty string
    std::string readToken() {
        // Fill tokenQueue if empty
        if (tokenQueue.empty()) {
            char buf[256];
            ssize_t n = read(fifoFd, buf, sizeof(buf));
            if (n > 0) {
                readBuffer.append(buf, n);
                size_t pos;
                // Process each complete line
                while ((pos = readBuffer.find('\n')) != std::string::npos) {
                    std::string line = readBuffer.substr(0, pos);
                    readBuffer.erase(0, pos + 1);
                    // Split into words and normalize
                    std::istringstream iss(line);
                    std::string word;
                    while (iss >> word) {
                        std::string norm = normalize(word);
                        if (!norm.empty()) tokenQueue.push_back(norm);
                    }
                }
            } else if (n < 0 && errno != EAGAIN) {
                std::cerr << "[Interpreter] read error: " << strerror(errno) << "\n";
            }
        }
        if (!tokenQueue.empty()) {
            std::string tok = tokenQueue.front();
            tokenQueue.pop_front();
            return tok;
        }
        return {};
    }

    void dispatchToken(const std::string& token) {
        auto now = std::chrono::steady_clock::now();
        // 1) Vehicle selection: ignore until we see a valid vehicle
        if (vehicles.count(token)) {
            if (inAction) finishAction();
            currentVehicle = token;
            std::cout << "[Interpreter] Vehicle set: " << token << "\n";
            return;
        }

        // If no vehicle selected yet, drop token
        if (currentVehicle.empty()) return;

        // 2) End keyword: finalize current action
        if (token == endKeyword) {
            if (inAction) finishAction();
            return;
        }

        // 3) Action keyword: start (or restart) an action
        if (validActions.count(token)) {
            if (inAction) {
                std::cout << "[Interpreter] New action '" << token
                          << "' detected, executing previous '" << currentAction << "' now.\n";
                finishAction();
            }
            currentAction     = token;
            actionArgs.clear();
            inAction          = true;
            actionStartTime   = now;
            std::cout << "[Interpreter] Action started: " << token << "\n";
            return;
        }

        // 4) Argument collection
        if (inAction) {
            actionArgs.push_back(token);
        }
    }

    void checkTimeout() {
        if (inAction) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - actionStartTime
            ).count();
            if (elapsed >= 5) {
                finishAction();
            }
        }
    }

    void finishAction() {
        auto it = vehicles.find(currentVehicle);
        if (it != vehicles.end()) {
            Command cmd(currentAction, actionArgs);
            it->second->validateCommand(cmd);
        }
        inAction = false;
    }

    std::string normalize(const std::string& in) {
        std::string out;
        out.reserve(in.size());
        for (char c : in) {
            if (std::isalnum((unsigned char)c)) out.push_back(std::tolower((unsigned char)c));
        }
        return out;
    }
};

