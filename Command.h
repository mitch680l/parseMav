#pragma once
#include <string>
#include <vector>

class Command {
    std::string action;
    std::vector<std::string> args;
    

public:
    Command(std::string a, std::vector<std::string> argList) : action(std::move(a)), args(std::move(argList)) {}
    std::string getAction() const { return action; }
    const std::vector<std::string>& getArgs() const { return args; }
    void printArgs() const;
};