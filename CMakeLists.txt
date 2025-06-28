cmake_minimum_required(VERSION 3.28.3)
project(geometry_common)

add_compile_options(-std=c++17)

# All warning enabled build option set to ON by default
option(ENABLE_ALL_WARNINGS "Build with all warnings enabled" ON)
if(ENABLE_ALL_WARNINGS)
    add_compile_options(-Wall)
endif(ENABLE_ALL_WARNINGS)

# Optimisation flag (O3) build option set to ON by default
option(BUILD_WITH_MAX_OPTIMISATION "Build with -O3 compile option" ON)
if(BUILD_WITH_MAX_OPTIMISATION)
    add_compile_options(-O3)
endif(BUILD_WITH_MAX_OPTIMISATION)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# The documentation build option set to OFF by default
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOC)

# =========
# LIBRARIES
# =========
add_library(geometry_utils
    src/Utils.cpp
    # data structures
    src/Point2D.cpp
    src/Point3D.cpp
    src/Polygon2D.cpp
    src/Polyline2D.cpp
    src/Pose2D.cpp
    src/XYTheta.cpp
    src/Circle.cpp
    src/Box2D.cpp
    src/Box3D.cpp
    src/LineSegment2D.cpp
    src/TransformMatrix2D.cpp
    src/TransformMatrix3D.cpp
)
target_include_directories(geometry_utils PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(geometry_utils PUBLIC
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

add_library(pointcloud_projector
    src/PointCloudProjector.cpp
)
target_include_directories(geometry_utils PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(pointcloud_projector
    geometry_utils
)

# =======
# INSTALL
# =======
install(
    TARGETS
        geometry_utils
        pointcloud_projector
    EXPORT
        export_geometry_common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY
        include/
    DESTINATION
        include/${PROJECT_NAME}
)

# =====
# TESTS
# =====
if (BUILD_TESTING)
    add_subdirectory(test)
endif ()

ament_export_dependencies(
    tf2
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs)

ament_export_targets(export_geometry_common)

ament_package()
