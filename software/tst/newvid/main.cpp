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

void test(RigPtr rig)
{
    std::cout << "Let's test" << std::endl;

    std::cout << "Open" << std::endl;
    if( rig->open() == false ) throw std::runtime_error("could not open rig!");

    int count = 0;

    const auto t0 = std::chrono::steady_clock::now();

    while( (std::chrono::steady_clock::now() - t0) < std::chrono::milliseconds(3000) )
    {
        //rig->trigger();

        Image im;
        rig->read(im);

        if(im.isValid())
        {
            std::cout << "=> VALID image ( frame_id = " << im.getFrameId() << " )" << std::endl;
            count++;
        }
        else
        {
            std::cout << "=> INVALID image" << std::endl;
        }
    }

    std::cout << "FPS = " << double(count)/3.0 << std::endl;

    std::cout << "Close" << std::endl;
    rig->close();
    arv_shutdown();
}

int main(int num_args, char** args)
{
    arv_update_device_list();

    if(arv_get_n_devices() != 2) throw std::runtime_error("there should be two cameras!");

    //RigPtr rig(new Rig({0}));
    RigPtr rig(new Rig({0, 1}));

    test(rig);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    test(rig);

    arv_shutdown();

    return 0;
}

