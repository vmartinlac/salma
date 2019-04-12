#pragma once

#include <string>
#include "ExternalTrigger.h"

class ArduinoTrigger : public ExternalTrigger
{
public:

  ArduinoTrigger();
  ~ArduinoTrigger() override;

  void setPathToSerialPort(const std::string& path);

  bool open() override;
  void close() override;
  void trigger() override;

protected:

  std::string mPath;
  int mFD;
};

typedef std::shared_ptr<ArduinoTrigger> ArduinoTriggerPtr;

