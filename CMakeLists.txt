cmake_minimum_required (VERSION 3.2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

project (Arucotest)

# compiler version checking does not work if CMAKE_CXX_COMPILER is not set
if (CMAKE_CXX_COMPILER)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)
    message("GCC version: ${GCC_VERSION}")
    if (GCC_VERSION VERSION_GREATER "5.0.0" OR GCC_VERSION VERSION_EQUAL "5.0.0")
            message(STATUS "GCC version >= 5.0, use CXX11_ABI=1")
    else()
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
    endif()
endif()

find_package(OpenCV)

set(SOURCES
    "aruco.cpp"
    "camera_single.cpp"
    "camera.cpp"
    )

set(HEADERS
    "aruco.h"
    "camera_single.h"
    "camera.h"
    )


add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(
    lmao
    arucotest.cpp
)

target_link_libraries(lmao Arucotest)


