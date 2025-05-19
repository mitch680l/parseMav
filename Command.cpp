#include "Command.h"
#include <iostream>

void Command::printArgs() const {
    std::cout << "[Command] Args:";
    for (const auto& a : args) {
        std::cout << " " << a;
    }
    std::cout << "\n";
}  