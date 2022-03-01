cmake_minimum_required(VERSION 3.13)

# allow version number in project command
cmake_policy(SET CMP0048 NEW)

# declare the project
project(cppgpio VERSION 1.0.0 DESCRIPTION "C++ >=11 GPIO library")

# force cpack to pick version numbers from this project, and not from a parent
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

set(CMAKE_CXX_STANDARD 20)

message(STATUS "CMake Generator: ${CMAKE_GENERATOR}")

if (CMAKE_GENERATOR MATCHES "Makefile")

	# single configuration setup - make sure we have a build type
	if (NOT CMAKE_BUILD_TYPE)
		# Release is the default build type
		set(CMAKE_BUILD_TYPE "Release")
	endif()

else()

	# clear any build type to make understood we build a multi configuration setup
	set(CMAKE_BUILD_TYPE "")

endif()

# we need threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# INPUT FILES

set(CMAKE_CONFIG
	cppgpio-config.cmake
)

set(LIBHEADERS
	include/cppgpio/buttons.hpp
	include/cppgpio/gpio.hpp
	include/cppgpio/i2c.hpp
	include/cppgpio/lcd.hpp
	include/cppgpio/output.hpp
)

set(CONVHEADER
	include/cppgpio.hpp
)

set(INTERNAL_HEADERS
	src/make_unique.hpp
	src/tools.hpp
)

set(SOURCES
	src/buttons.cpp
	src/gpio.cpp
	src/i2c.cpp
	src/lcd.cpp
	src/output.cpp
	src/tools.cpp
)

# BUILD

add_library(cppgpio SHARED ${INTERNAL_HEADERS} ${LIBHEADERS} ${SOURCES})
add_library(cppgpio-static STATIC ${INTERNAL_HEADERS} ${LIBHEADERS} ${SOURCES})
add_library(cppgpio::cppgpio ALIAS cppgpio)
add_library(cppgpio::cppgpio-static ALIAS cppgpio-static)

# this lib is not intended for Windows, so let's just call both libs
# by the same name (but with different extensions, .so and .a) ..
# (with Windows this would collide for the separate .lib files)
set_target_properties(cppgpio-static PROPERTIES OUTPUT_NAME cppgpio)

target_link_libraries(cppgpio Threads::Threads)
target_link_libraries(cppgpio-static Threads::Threads)

target_include_directories(cppgpio
	PUBLIC
	$<INSTALL_INTERFACE:include>
	PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(cppgpio-static
	PUBLIC
	$<INSTALL_INTERFACE:include>
	PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# INSTALLATION

# always put this first, otherwise it risks to remove other already installed cmake config files
install(EXPORT cppgpio DESTINATION lib/cppgpio)
install(TARGETS cppgpio cppgpio-static EXPORT cppgpio DESTINATION lib)
install(FILES ${CMAKE_CONFIG} DESTINATION lib/cppgpio)
install(FILES ${LIBHEADERS} DESTINATION include/cppgpio)
install(FILES ${CONVHEADER} DESTINATION include)

set(CPACK_SOURCE_GENERATOR ZIP)
set(CPACK_SOURCE_PACKAGE_FILE_NAME ${PROJECT_NAME}-${CPACK_PACKAGE_VERSION})
set(CPACK_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION ON)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
include(CPack)
