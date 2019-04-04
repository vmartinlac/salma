#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include "SyncIO.h"

bool syncimwrite(const std::string& path, const cv::Mat& image)
{
    std::vector<uint8_t> data;
    int fd = -1;
    bool ok = true;

    if(ok)
    {
        //ok = cv::imencode(ext, image, data);
        throw std::runtime_error("TODO"); // TODO
    }

    if(ok)
    {
        fd = open( path.c_str(), O_WRONLY|O_SYNC|O_CREAT, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH );
        ok = (fd >= 0);
    }

    if(ok)
    {
        const ssize_t bytes = write(fd, &data.front(), data.size());
        ok = (bytes == data.size());
    }

    if(fd >= 0)
    {
        close(fd);
    }

    return ok;
}

