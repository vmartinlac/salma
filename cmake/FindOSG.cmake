find_package(OpenSceneGraph COMPONENTS osg osgGA osgViewer osgDB REQUIRED)
add_library(OpenSceneGraph INTERFACE)
target_include_directories(OpenSceneGraph INTERFACE ${OPENSCENEGRAPH_INCLUDE_DIRS})
target_link_libraries(OpenSceneGraph INTERFACE ${OPENSCENEGRAPH_LIBRARIES})
