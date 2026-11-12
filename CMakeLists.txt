# ###############################################################################
# MIT License
#
# Copyright (c) 2026 Nico Trost
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ###############################################################################

cmake_minimum_required(VERSION 3.22)
project(PowerBoxSDK VERSION 1.6.0 DESCRIPTION "PowerBox SDK")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find libudev package
if(UNIX)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUDEV libudev)
endif()

# SDK sources (conditionally include PinsBox / GPIO manager)
set(SDK_SOURCES
    PowerBoxSDK.cpp
    PowerBoxLogging.cpp
    PowerBoxSerialPort.cpp
    PowerBoxDevice.cpp
    DS18B20.cpp
    Device.cpp
    USB.cpp
    ADJ.cpp
    base64.cpp
    MCP4725.cpp
    INA219.cpp
    PWM.cpp
    DHT22.cpp
    DEW.cpp
    BTS7XXX.cpp
    MCP320X.cpp
    TCA953X.cpp)

# Detect libgpiod (v2 API, >= 2.0) and append hardware sources when available
if(UNIX)
    pkg_check_modules(LIBGPIOD libgpiod>=2.0)
endif()

if(LIBGPIOD_FOUND)
    list(APPEND SDK_SOURCES GPIOManager.cpp PinsBoxDevice.cpp PinsBoxLightDevice.cpp PinsBoxMiniDevice.cpp PinsBoxCustomDevice.cpp StellaVitaDevice.cpp ASIAirDevice.cpp)
else()
    message(STATUS "libgpiod >= 2.0 not found. GPIO / PinsBox support will be disabled (install libgpiod-dev >= 2.0 to enable)")
endif()

# SDK Library
add_library(PowerBoxSDK SHARED ${SDK_SOURCES})

# Public headers
target_include_directories(PowerBoxSDK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Find and link OpenSSL
find_package(OpenSSL REQUIRED)
target_link_libraries(PowerBoxSDK PRIVATE OpenSSL::Crypto)

# Find and link jsoncpp
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
target_include_directories(PowerBoxSDK PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(PowerBoxSDK PRIVATE ${JSONCPP_LIBRARIES})

# Link libudev
if(UNIX)
    if(LIBUDEV_FOUND)
        target_include_directories(PowerBoxSDK PRIVATE ${LIBUDEV_INCLUDE_DIRS})
        target_link_libraries(PowerBoxSDK PRIVATE ${LIBUDEV_LIBRARIES})
    else()
        message(FATAL_ERROR "libudev not found. Install it with: sudo apt-get install libudev-dev")
    endif()
endif()

# Find and link libgpiod
if(UNIX)
    pkg_check_modules(LIBGPIOD libgpiod>=2.0)
    if(LIBGPIOD_FOUND)
        target_compile_definitions(PowerBoxSDK PRIVATE HAVE_LIBGPIOD=1)
        target_include_directories(PowerBoxSDK PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
        target_link_libraries(PowerBoxSDK PRIVATE ${LIBGPIOD_LIBRARIES})
    else()
        message(STATUS "libgpiod >= 2.0 not found. GPIO control may not work. Install with: sudo apt-get install libgpiod-dev")
    endif()
endif()

if(WIN32)
    target_link_libraries(PowerBoxSDK PRIVATE SetupAPI)
    # Prevent Windows headers from defining min/max macros
    target_compile_definitions(PowerBoxSDK PRIVATE NOMINMAX)
endif()

# Test executables
add_executable(test_power_box test_power_box.cpp)
target_link_libraries(test_power_box PowerBoxSDK)

# Installation
install(TARGETS PowerBoxSDK
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(FILES PowerBoxSDK.h PowerBoxLogging.h PowerBoxSerialPort.h PowerBoxDevice.h PinsBoxDevice.h PinsBoxLight.h PinsBoxMiniDevice.h DS18B20.h DHT22.h ADJ.h DEW.h PWM.h SPIBus.h MCP320X.h GPIOManager.h Device.h USB.h MCP4725.h TCP953X.h INA219.h BTS7XXX.h DESTINATION include)

# CPack configuration for Debian package generation
if(UNIX)
    set(CPACK_GENERATOR "DEB")
    set(CPACK_PACKAGE_CONTACT "Nico")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/nitr57/PowerBoxSDK")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Nico")
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION_SUMMARY "SDK for controlling Power Box devices via serial communication")

    # Runtime dependencies
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1")

    # License
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

    include(CPack)
endif()
