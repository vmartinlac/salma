#include "ArduinoTrigger.h"

ArduinoTrigger::ArduinoTrigger()
{
}

ArduinoTrigger::~ArduinoTrigger()
{
}

void ArduinoTrigger::setPathToSerialPort(const std::string& path)
{
    mPath = path;
}

bool ArduinoTrigger::open()
{
    mSerialPort.open(mPath.c_str());
    return mSerialPort.is_open();
}

void ArduinoTrigger::close()
{
    mSerialPort.close();
}

void ArduinoTrigger::trigger()
{
    //std::cout << "trigger" << std::endl;
    mSerialPort << "T" << std::flush;
}

