#pragma once

#include <QThread>

class Camera;

class SLAMEngine : public QThread
{
public:

    static SLAMEngine* create(Camera* camera);

    virtual ~SLAMEngine();
};

