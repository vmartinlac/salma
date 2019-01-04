
add_library(
    salma_mvpnp
    SHARED
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
    salma_mvpnp
    PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    opencv_core
    opencv_calib3d)

target_include_directories(salma_mvpnp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS salma_mvpnp LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

