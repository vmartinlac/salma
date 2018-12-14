
add_library(
    mvpnp
    STATIC
    MVPnP.cpp
    MVPnP.h
    MVPnPLM.cpp
    MVPnPLM.h
    MVPnPRANSACLM.cpp
    MVPnPRANSACLM.h
    MVPnPMonoOpenCV.cpp
    MVPnPMonoOpenCV.h
)

target_link_libraries(
    mvpnp
    PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    opencv_core
    opencv_calib3d)

target_include_directories(mvpnp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

