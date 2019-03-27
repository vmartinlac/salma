#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>
#include <arv.h>
#include <opencv2/core.hpp>

#include "Rig.h"
#include "Camera.h"

int main(int num_args, char** args)
{
    arv_update_device_list();

    if(arv_get_n_devices() != 2) throw std::runtime_error("there should be two cameras!");

    RigPtr rig(new Rig({0, 1}));

    rig->open();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    rig->close();

    arv_shutdown();

    return 0;
}

