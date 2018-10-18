#include "ArduinoTrigger.h"

ArduinoTrigger::ArduinoTrigger(const std::string& rs232_filename)
{
    mRS232Path = rs232_filename;
}

ArduinoTrigger::~ArduinoTrigger()
{
    close();
}

bool ArduinoTrigger::open()
{
    mFile.open(mRS232Path.c_str(), std::ofstream::out);
    return mFile.is_open();
}

void ArduinoTrigger::close()
{
    mFile.close();
}

void ArduinoTrigger::trigger()
{
    mFile << "T";
}

