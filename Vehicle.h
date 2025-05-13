#pragma once
#include <string>
#include <memory>
#include "Command.h"
#include <set>

class Vehicle {
public:
    virtual ~Vehicle() = default;
    virtual std::string getName() const = 0;
    virtual void executeCommand(const Command& cmd) = 0;
    virtual bool validateCommand(const Command& cmd) const = 0;
};