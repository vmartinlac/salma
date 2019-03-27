add_subdirectory(testmvpnp)
add_subdirectory(newvid)
if(${SALMA_WITH_CUDA})
    add_subdirectory(teststereomatching)
endif()
add_subdirectory(testtriangulation)
add_subdirectory(testlba)

