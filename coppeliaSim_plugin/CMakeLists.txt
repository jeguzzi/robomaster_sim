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

option(ARCHIVE "Create a TAR.GZ archive using CPACK" OFF)

if(ARCHIVE)
  if(APPLE)
    set(COPPELIASIM_LUA_DIR Contents/Resources/lua)
    set(COPPELIASIM_MODELS_DIR Contents/Resources/models)
    set(COPPELIASIM_MANUAL_DIR Contents/Resources/manual)
    set(COPPELIASIM_PLUGINS_DIR Contents/MacOS)
  else()
    set(COPPELIASIM_LUA_DIR lua)
    set(COPPELIASIM_MODELS_DIR models)
    set(COPPELIASIM_MANUAL_DIR manual)
    set(COPPELIASIM_PLUGINS_DIR .)
  endif()
endif()

if(${COPPELIASIM_VERSION} VERSION_GREATER_EQUAL 4.8)
  set(MODEL_VERSION 2)
else()
  set(MODEL_VERSION 1)
endif()

set(MODEL_FOLDER ${COPPELIASIM_MODELS_DIR}/robots/mobile)
set(MODEL_SENSOR_FOLDER ${COPPELIASIM_MODELS_DIR}/components/sensors)

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

if(WIN32)
  execute_process(
    COMMAND
      powershell -Command
      "(gc ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml) -replace 'PLUGIN_NAME', '${_PLUGIN_NAME_XML}' | Out-File -encoding ASCII callbacks.xml"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
else()
  execute_process(
    COMMAND
      bash -c
      "sed s/PLUGIN_NAME/${_PLUGIN_NAME_XML}/g ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml > callbacks.xml"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

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

if(ARCHIVE)
  set(CPACK_GENERATOR TGZ)
  set(CPACK_PACKAGE_NAME "simRobomaster")
  set(CPACK_OUTPUT_FILE_PREFIX "archives")
  set(CPACK_PACKAGE_FILE_NAME
      ${CPACK_PACKAGE_NAME}-${CMAKE_PROJECT_VERSION_MAJOR}.${CMAKE_PROJECT_VERSION_MINOR}-CoppeliaSim-${COPPELIASIM_VERSION_MAJOR}.${COPPELIASIM_VERSION_MINOR}-${CMAKE_SYSTEM_NAME}
  )
  include(CPack)
endif()
