#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "ArduinoTrigger.h"

ArduinoTrigger::ArduinoTrigger()
{
    mFD = -1;
}

ArduinoTrigger::~ArduinoTrigger()
{
    if(mFD >= 0)
    {
        close();
    }
}

void ArduinoTrigger::setPathToSerialPort(const std::string& path)
{
    mPath = path;
}

bool ArduinoTrigger::open()
{
    struct stat info;
    bool ok = true;

    if(ok)
    {
        mFD = ::open(mPath.c_str(), O_WRONLY);
        ok = (mFD >= 0);
    }

    if(ok)
    {
        ok = (0 == ::fstat(mFD, &info));
    }

    if(ok)
    {
        ok = S_ISCHR(info.st_mode);
    }

    if(ok == false && mFD >= 0)
    {
        ::close(mFD);
    }

    return ok;
}

void ArduinoTrigger::close()
{
    if(mFD >= 0)
    {
        ::close(mFD);
        mFD = -1;
    }
}

void ArduinoTrigger::trigger()
{
    if(mFD >= 0)
    {
        const char buff = 'T';
        write(mFD, &buff, 1);
    }
}

