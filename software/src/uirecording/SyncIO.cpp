#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <turbojpeg.h>
#include "SyncIO.h"

bool syncimwrite(const std::string& path, const cv::Mat& image)
{
    int fd = -1;
    bool ok = true;

    unsigned long buff_size = 0;
    unsigned char* buff = nullptr;

    tjhandle handle = nullptr;

    if(ok)
    {
        ok = ( image.type() == CV_8UC3 && bool( image.data ) && image.cols > 0 && image.rows > 0 );
    }

    if(ok)
    {
        handle = tjInitCompress();
        ok = bool(handle);
    }

    if(ok)
    {
        const int ret = tjCompress2(
            handle, image.data, image.cols, image.step1(), image.rows, TJPF_BGR,
            &buff, &buff_size, TJSAMP_444, 100, TJFLAG_ACCURATEDCT);

        ok = (ret == 0);
    }

    if(ok)
    {
        fd = open( path.c_str(), O_WRONLY|O_SYNC|O_CREAT, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH );
        ok = (fd >= 0);
    }

    if(ok)
    {
        //const ssize_t bytes = write(fd, &data.front(), data.size());
        //ok = (bytes == data.size());

        const ssize_t bytes = write(fd, buff, buff_size);
        ok = (bytes == buff_size);
    }

    if(fd >= 0)
    {
        close(fd);
    }

    if(buff)
    {
        tjFree(buff);
    }

    if(handle)
    {
        tjDestroy(handle);
    }

    return ok;
}

