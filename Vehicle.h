#pragma once
#include <string>
#include <memory>
#include "Command.h"

class Vehicle {
public:
    virtual ~Vehicle() = default;
    virtual std::string getName() const = 0;
    virtual void executeCommand(const Command& cmd) = 0;
};