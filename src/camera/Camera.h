#pragma once

#include <string>

class Image;

class Camera
{

public:

    Camera();
    virtual ~Camera();

    virtual std::string getHumanName() = 0;

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual void read(Image& image) = 0;
};

