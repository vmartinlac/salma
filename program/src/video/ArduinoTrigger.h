#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include "Trigger.h"

class ArduinoTrigger : public Trigger
{
public:

    ArduinoTrigger(const std::string& rs232_filename);
    ~ArduinoTrigger();

    bool open() override;
    void close() override;

    void trigger() override;

protected:

    std::string mRS232Path;
    std::ofstream mFile;
};

