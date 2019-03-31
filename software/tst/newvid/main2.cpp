#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include "Rig.h"
#include "Camera.h"

VmbError_t DiscoverGigECameras()
{
    VmbError_t  err     = VmbErrorSuccess;
    VmbBool_t isGigE  = VmbBoolFalse;

    err = VmbFeatureBoolGet( gVimbaHandle, "GeVTLIsPresent", &isGigE );                 // Is Vimba connected to a GigE transport layer?
    if ( VmbErrorSuccess == err )
    {
        if( VmbBoolTrue == isGigE )
        {
            err = VmbFeatureIntSet( gVimbaHandle, "GeVDiscoveryAllDuration", 250 );     // Set the waiting duration for discovery packets to return. If not set the default of 150 ms is used.
            if( VmbErrorSuccess == err )
            {
                err = VmbFeatureCommandRun( gVimbaHandle, "GeVDiscoveryAllOnce" );      // Send discovery packets to GigE cameras and wait 250 ms until they are answered
                if( VmbErrorSuccess != err )
                {
                    printf( "Could not ping GigE cameras over the network. Reason: %d\n", err );
                }
            }
            else
            {
                printf( "Could not set the discovery waiting duration. Reason: %d\n", err );
            }
        }
    }
    else
    {
        printf( "Could not query Vimba for the presence of a GigE transport layer. Reason: %d\n\n", err );
    }

    return err;
}

void test(RigPtr rig)
{
    std::cout << "Let's test" << std::endl;

    std::cout << "Open" << std::endl;
    rig->open();

    int count = 0;

    const auto t0 = std::chrono::steady_clock::now();

    while( (std::chrono::steady_clock::now() - t0) < std::chrono::milliseconds(3000) )
    {
        //rig->trigger();

        Image im;
        rig->read(im);

        if(im.isValid())
        {
            //std::cout << "=> VALID image ( frame_id = " << im.getFrameId() << " )" << std::endl;
            count++;
        }
        else
        {
            //std::cout << "=> INVALID image" << std::endl;
        }
    }

    std::cout << "FPS = " << double(count)/3.0 << std::endl;

    std::cout << "Close" << std::endl;
    rig->close();
}

#define CHECK_VMBRET(x) { if( (x) != VmbErrorSuccess ) throw std::runtime_error("vimba_failed"); }

void todo()
{
    VmbUint32_t num_cameras = 0;
    std::vector<VmbCameraInfo_t> camera_infos;

    VmbStartup();

    DiscoverGigECameras();

    CHECK_VMBRET( VmbCamerasList(nullptr, 0, &num_cameras, 0) );
    std::cout << "Found " << num_cameras << " cameras!" << std::endl;

    if(num_cameras)
    {
        camera_infos.resize(num_cameras);
        VmbCamerasList(&camera_infos.front(), num_cameras, &num_cameras, sizeof(VmbCameraInfo_t));

        for(int i=0; i<num_cameras; i++)
        {
            std::cout << camera_infos[i].cameraName << std::endl;
        }
    }

    if( num_cameras < 2 ) throw std::runtime_error("Need at least two cameras!");

    std::vector<std::string> ids({
        camera_infos[0].cameraIdString,
        camera_infos[1].cameraIdString
    });

    RigPtr rig(new Rig(ids));
    //RigPtr rig(new Rig({0, 1}));

    test(rig);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    test(rig);

    VmbShutdown();
}

int main(int num_args, char** args)
{
    std::thread t(todo);
    t.join();
    return 0;
}

