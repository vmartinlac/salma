#pragma once

#include <memory>

class ExternalTrigger
{
public:

  ExternalTrigger();
  virtual ~ExternalTrigger();

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual void trigger() = 0;

};

typedef std::shared_ptr<ExternalTrigger> ExternalTriggerPtr;

