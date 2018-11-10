#pragma once

#include <string>

enum TriggerMode
{
    TRIGGER_SOFTWARE,
    TRIGGER_ARDUINO
};

struct TriggerInfo
{
    TriggerInfo();

    TriggerMode mode;
    std::string path;
};

