#pragma once
#include <exception>

class NotImplementedException : public std::exception
{
public:
    NotImplementedException(const char* message = "Function not yet implemented")
        : std::exception(message) {}
};
