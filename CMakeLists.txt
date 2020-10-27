cmake_minimum_required (VERSION 2.8.1)

######## Project settings ########
PROJECT(NPWEdge)
SET(LICENSE "TBD")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} 
#add_compile_options("-zmuldefs -DNDEBUG -g")
add_compile_options("-zmuldefs -g")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

######## Build and include settings ########
include_directories(
	include
)

link_directories(
	${LINK_DIRECTORIES}
)


file(GLOB SOURCES
    "./src/*.cpp"
    "./tests/*.cpp"
)

add_executable(
	NPWEdge

	${SOURCES}
)

set_target_properties(
	NPWEdge
	
	PROPERTIES
	CXX_STANDARD 14)

TARGET_LINK_LIBRARIES(
	NPWEdge

	glog
	paho-mqttpp3
	paho-mqtt3as
	pthread
	serial
    stdc++fs
    modbus
)

######## Install targets ########
#INSTALL(TARGETS NPWEdge
#	RUNTIME DESTINATION bin
#)
