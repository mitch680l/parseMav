#pragma once
#include <string>
#include <vector>

/*
Command stores <action, args<>> pairs. Args is a vector of all potential arguments.
This is effectively the interface between the parser and vehicle. Where the parser gives the vehicle a possible command, 
however its up to the vehicles implementation to determine if the command is valid or not.

Example: <"move", ["forward", "10"]>
*/

class Command {
    std::string action;
    std::vector<std::string> args;
    

public:
    Command(std::string a, std::vector<std::string> argList) : action(std::move(a)), args(std::move(argList)) {}
    std::string getAction() const { return action; }
    const std::vector<std::string>& getArgs() const { return args; }
    void printArgs() const;
};