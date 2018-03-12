PROJECT(footbot_controller)
cmake_minimum_required(VERSION 3.5)

set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake /usr/share/argos3/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

include(/usr/local/share/argos3/cmake/ARGoSCheckQTOpenGL.cmake)

find_package(Boost 1.66.0 COMPONENTS system REQUIRED)
find_package(Lua REQUIRED)

link_directories(${ARGOS_LIBRARY_DIRS} /usr/local/lib/argos3/)

include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_library(footbot_ai_controller MODULE footbot_ai_controller.h footbot_ai_controller.cpp)
target_link_libraries(footbot_ai_controller
  argos3core_simulator.so
  argos3plugin_simulator_footbot.so
  argos3plugin_simulator_genericrobot.so
  ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${Boost_LIBRARIES})

MESSAGE( STATUS "QT_LIBRARIES -> : " ${QT_LIBRARIES} )
MESSAGE( STATUS "GLUT_LIBRARY -> : " ${GLUT_LIBRARY} )
MESSAGE( STATUS "OPENGL_LIBRARY -> : " ${OPENGL_LIBRARY} )
