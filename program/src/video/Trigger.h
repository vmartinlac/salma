#pragma once

#include <memory>

class Trigger
{
public:

    Trigger();
    virtual ~Trigger();

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual void trigger() = 0;
};

typedef std::shared_ptr<Trigger> TriggerPtr;

