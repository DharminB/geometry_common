find_package(ament_cmake_gtest REQUIRED)

# NOTE: all test must end with "_test.cpp" !
FILE(GLOB TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*_test.cpp" )

ament_add_gtest(geometry_common_test
    main.cpp
    ${TEST_SOURCES}
)
target_link_libraries(geometry_common_test
    geometry_utils
)
