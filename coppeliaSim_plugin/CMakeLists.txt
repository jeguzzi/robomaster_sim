set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

if(NOT COPPELIASIM_INCLUDE_DIR)
  if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
    set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
  else()
    set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
  endif()
endif()
if(NOT LIBPLUGIN_DIR)
  if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
    set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
  else()
    set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
     ${COPPELIASIM_INCLUDE_DIR}/cmake ${LIBPLUGIN_DIR}/cmake)

find_package(CoppeliaSim 4 REQUIRED)

if(${COPPELIASIM_VERSION} VERSION_GREATER_EQUAL 4.8)
  set(MODEL_VERSION 2)
else()
  set(MODEL_VERSION 1)
endif()

set(MODEL_FOLDER $ENV{COPPELIASIM_ROOT_DIR}/models/robots/mobile)
set(MODEL_SENSOR_FOLDER $ENV{COPPELIASIM_ROOT_DIR}/models/components/sensors)

if(${COPPELIASIM_VERSION} VERSION_GREATER_EQUAL 4.6)
  set(_PLUGIN_NAME simRobomaster)
  set(_PLUGIN_NAME_XML simRobomaster)
else()
  set(_PLUGIN_NAME simExtRobomaster)
  set(_PLUGIN_NAME_XML Robomaster)
endif()

set(COPPELIASIM_VERSION
    ${COPPELIASIM_VERSION}
    PARENT_SCOPE)
set(_PLUGIN_NAME
    ${_PLUGIN_NAME}
    PARENT_SCOPE)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)

execute_process(
  COMMAND
    bash -c
    "sed s/PLUGIN_NAME/${_PLUGIN_NAME_XML}/g ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml > callbacks.xml"
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

coppeliasim_generate_stubs(
  ${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE
  ${CMAKE_CURRENT_BINARY_DIR}/callbacks.xml LUA_FILE
  ${CMAKE_CURRENT_SOURCE_DIR}/simRobomaster.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/config.h ESCAPE_QUOTES)

coppeliasim_add_plugin(${_PLUGIN_NAME} SOURCES plugin.cpp coppeliasim_robot.cpp)

install(FILES ../models/v${MODEL_VERSION}/RoboMasterEP.ttm
              ../models/v${MODEL_VERSION}/RoboMasterS1.ttm
        DESTINATION ${MODEL_FOLDER})
install(FILES ../models/v${MODEL_VERSION}/RoboMasterDistanceSensor.ttm
        DESTINATION ${MODEL_SENSOR_FOLDER})

target_compile_definitions(${_PLUGIN_NAME}
                           PRIVATE MODEL_VERSION=${MODEL_VERSION})
