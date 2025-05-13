#pragma once
#include <map>
#include <string>
#include <memory>
#include "Vehicle.h"
#include "Rover.h"
#include "Plane.h"
#include "Copter.h"
#include <fstream>
#include <set>
#include <sstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

class SpeechInterpreter {
    std::map<std::string, std::unique_ptr<Vehicle>> vehicles;
    std::string fifoPath;
    std::set<std::string> validActions;

    void loadActionsFromFile(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty()) validActions.insert(line);
        }
    }

    std::vector<std::string> split(const std::string& str) {
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::string word;
        while (iss >> word) tokens.push_back(word);
        return tokens;
    }

public:
    explicit SpeechInterpreter(const std::string& path) : fifoPath(path) {
        vehicles["Rover"] = std::make_unique<Rover>();
        vehicles["Plane"] = std::make_unique<Plane>();
        vehicles["Copter"] = std::make_unique<Copter>();
        loadActionsFromFile("actions.cfg");
    }

    void run() {
        mkfifo(fifoPath.c_str(), 0666);
        char buffer[1024];
        while (true) {
            int fd = open(fifoPath.c_str(), O_RDONLY);
            ssize_t n = read(fd, buffer, sizeof(buffer) - 1);
            if (n > 0) {
                buffer[n] = '\0';
                std::string input(buffer);
                auto tokens = split(input);
                if (tokens.empty()) continue;

                auto it = vehicles.find(tokens[0]);
                if (it != vehicles.end()) {
                    size_t i = 1;
                    while (i < tokens.size()) {
                        if (validActions.count(tokens[i])) {
                            std::string action = tokens[i];
                            size_t j = i + 1;
                            while (j < tokens.size() && !validActions.count(tokens[j])) {
                                ++j;
                            }
                            std::vector<std::string> args(tokens.begin() + i + 1, tokens.begin() + j);
                            Command cmd(action, args);
                            if (it->second->validateCommand(cmd)) {
                                it->second->executeCommand(cmd);
                            } else {
                                std::cout << "[Interpreter] Invalid arguments for command '" << action << "'\n";
                            }
                            i = j;
                        } else {
                            ++i;
                        }
                    }
                }
            }
            close(fd);
        }
    }
};