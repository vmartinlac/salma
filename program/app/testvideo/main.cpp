#include "VideoSystem.h"

int main(int num_args, char** args)
{
    VideoSystem::instance()->initialize();
    VideoSystem::instance()->finalize();

    return 0;
}
