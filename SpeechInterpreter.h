#pragma once
#include <map>
#include <string>
#include <memory>
#include "Vehicle.h"
#include "Rover.h"
#include "Plane.h"
#include "Copter.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

class SpeechInterpreter {
    std::map<std::string, std::unique_ptr<Vehicle>> vehicles;
    std::string fifoPath;

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
                    std::string action = (tokens.size() > 1) ? tokens[1] : "";
                    std::vector<std::string> args(tokens.begin() + 2, tokens.end());
                    Command cmd(action, args);
                    it->second->executeCommand(cmd);
                }
            }
            close(fd);
        }
    }
};