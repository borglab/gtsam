# CMake file for cpp libraries and projects
#
# To use:
#   cmake_minimum_required(VERSION 2.6)
#   include("$ENV{HOME}/borg/gtsam/buildtools/gt.cmake")
#   project(your-project-name CXX C)
#
# November, 2009 - Richard Roberts

cmake_policy(PUSH)

cmake_minimum_required(VERSION 2.6)

# Set the default install prefix
IF(NOT CMAKE_INSTALL_PREFIX OR CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")
  SET(CMAKE_INSTALL_PREFIX "$ENV{HOME}" CACHE PATH "Installation prefix" FORCE)
ENDIF()

# Set the default build type
IF(NOT DEFINED CMAKE_BUILD_TYPE)
  message(STATUS "[gt.cmake] CMAKE_BUILD_TYPE not defined, defaulting to Debug")
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF()

# Add borg libraries and includes
include_directories("${CMAKE_INSTALL_PREFIX}/include")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")

# Add source directory as an include directory so installed header paths match project
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# Path to CppUnitLite
set(BORG_SOFTWARE_ROOT "$ENV{HOME}/borg")
set(GT_CPPUNITLITE_INCLUDE_DIR "${BORG_SOFTWARE_ROOT}/gtsam")
set(GT_CPPUNITLITE_LIB_DIR "${BORG_SOFTWARE_ROOT}/gtsam/CppUnitLite")

# Enable unit testing
enable_testing()
add_custom_target(check make all test)


############
### Quick "use" functions
file(TO_CMAKE_PATH "${CMAKE_CURRENT_LIST_FILE}" GT_BUILDTOOLS)
string(REPLACE "/" ";" GT_BUILDTOOLS "${GT_BUILDTOOLS}")
list(REMOVE_AT GT_BUILDTOOLS -1)
string(REPLACE ";" "/" GT_BUILDTOOLS "${GT_BUILDTOOLS}")
message(STATUS "[gt.cmake] Build tools dir ${GT_BUILDTOOLS}")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${GT_BUILDTOOLS}")
function(GT_USE_BOOST)
    if(ARGN)
        find_package(Boost 1.40 REQUIRED COMPONENTS ${ARGN})
    else()
        find_package(Boost 1.40 REQUIRED)
    endif()
    link_directories(${Boost_LIBRARY_DIRS})
    include_directories(${Boost_INCLUDE_DIRS})
    link_libraries(${Boost_LIBRARIES})
endfunction(GT_USE_BOOST)
function(GT_USE_QT4)
    find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui ${ARGN})
    include(${QT_USE_FILE})
    link_libraries(${QT_LIBRARIES})
endfunction(GT_USE_QT4)
function(GT_USE_OPENCV)
    find_package(OpenCV REQUIRED)
    link_directories(${OpenCV_LINK_DIRECTORIES})
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBRARIES)
endfunction(GT_USE_OPENCV)
function(GT_USE_IPP)
    if(ARGN)
        find_package(IPP REQUIRED COMPONENTS ${ARGN})
    else()
        find_package(IPP REQUIRED)
    endif()
    link_directories(${IPP_LIBRARY_PATHS})
    include_directories(${IPP_INCLUDE_PATHS})
    link_libraries(${IPP_SHARED_LIBRARIES})
endfunction()
function(GT_USE_GTSAM)
    link_libraries("gtsam")
endfunction(GT_USE_GTSAM)
function(GT_USE_EASY2D)
    link_libraries("easy2d")
endfunction(GT_USE_EASY2D)


############
### Main target functions, only call one of these.

# Set the "excluded" sources, i.e. unit tests
function(GT_EXCLUDE)
    set(EXCLUDE_SOURCES "")
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        set(EXCLUDE_SOURCES ${EXCLUDE_SOURCES} ${srcs})
        #message(STATUS "Excluding sources: " ${srcs})
    endforeach(srcpat)
    set(GT_EXCLUDE_SOURCES ${GT_EXCLUDE_SOURCES} ${EXCLUDE_SOURCES} PARENT_SCOPE)
endfunction(GT_EXCLUDE)

# Define "common" sources when there is no main library or executable
function(GT_MAIN_SOURCES)
    gt_main_sources_helper(${ARGN})
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
    # Build a "convenience" static library
    message(STATUS "[gt.cmake] Adding convenience library \"${PROJECT_NAME}\" with sources ${GT_COMMON_SOURCES}")
    add_library(${PROJECT_NAME}-static STATIC ${GT_COMMON_SOURCES})
    SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
    SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
endfunction(GT_MAIN_SOURCES)
function(GT_MAIN_SOURCES_HELPER)
    set(GT_COMMON_SOURCES "")
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        set(GT_COMMON_SOURCES ${GT_COMMON_SOURCES} ${srcs})
        #message(STATUS "Adding sources: " ${srcs})
    endforeach(srcpat)
    if(GT_EXCLUDE_SOURCES)
        list(REMOVE_ITEM GT_COMMON_SOURCES ${GT_EXCLUDE_SOURCES})
    endif(GT_EXCLUDE_SOURCES)
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
endfunction(GT_MAIN_SOURCES_HELPER)

# Add headers to be installed
function(GT_INSTALL_HEADERS)
    list(GET ARGN 0 arg1)
    if(${arg1} STREQUAL "PREFIX")
        list(GET ARGN 1 prefix)
        list(REMOVE_AT ARGN 0 1)
    else(${arg1} STREQUAL "PREFIX")
        set(prefix "")
    endif(${arg1} STREQUAL "PREFIX")
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        install(FILES ${srcs} DESTINATION "include/${PROJECT_NAME}/${prefix}")
    endforeach(srcpat)
endfunction(GT_INSTALL_HEADERS)

# Define the main shared+static library from the given sources
function(GT_MAIN_LIB)
    gt_main_sources_helper(${ARGN})
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
    message(STATUS "[gt.cmake] Adding main lib \"${PROJECT_NAME}\" with sources ${GT_COMMON_SOURCES}")
    add_library(${PROJECT_NAME}-shared SHARED ${GT_COMMON_SOURCES})
    #add_library(${PROJECT_NAME}-static STATIC ${GT_COMMON_SOURCES})
    SET_TARGET_PROPERTIES(${PROJECT_NAME}-shared PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
    SET_TARGET_PROPERTIES(${PROJECT_NAME}-shared PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    #SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    install(TARGETS ${PROJECT_NAME}-shared LIBRARY DESTINATION "lib" ARCHIVE DESTINATION "lib")
endfunction(GT_MAIN_LIB)

# Define the main static library from the given sources
function(GT_MAIN_STLIB)
    gt_main_sources(${ARGN})
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
    message(STATUS "[gt.cmake] Adding main stlib \"${PROJECT_NAME}\" with sources ${GT_COMMON_SOURCES}")
    #add_library(${PROJECT_NAME}-static STATIC ${GT_COMMON_SOURCES})
    #SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
    #SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    #add_library(${PROJECT_NAME} STATIC ${GT_COMMON_SOURCES})
    install(TARGETS ${PROJECT_NAME}-static ARCHIVE DESTINATION "lib")
endfunction(GT_MAIN_STLIB)

# Set the build version for the main library
function(GT_MAIN_VERSION version)
    if(TARGET ${PROJECT_NAME}-shared)
        set_target_properties(${PROJECT_NAME}-shared PROPERTIES VERSION "${version}")
    endif()
    if(TARGET ${PROJECT_NAME}-static)
        set_target_properties(${PROJECT_NAME}-static PROPERTIES VERSION "${version}")
    endif()
endfunction()

# Set the API version for the main library
function(GT_MAIN_SOVERSION soversion)
    if(TARGET ${PROJECT_NAME}-shared)
        set_target_properties(${PROJECT_NAME}-shared PROPERTIES SOVERSION "${soversion}")
    endif()
    if(TARGET ${PROJECT_NAME}-static)
        set_target_properties(${PROJECT_NAME}-static PROPERTIES SOVERSION "${soversion}")
    endif()
endfunction()

# Define the main executable from the given sources
#function(GT_MAIN_EXE)
#    gt_main_sources(${ARGN})
#    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
#    message(STATUS "[gt.cmake] Adding main exe \"${PROJECT_NAME}\" with sources ${GT_COMMON_SOURCES}")
#    add_executable(${PROJECT_NAME} ${GT_COMMON_SOURCES})
#    install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION "bin")
#endfunction(GT_MAIN_EXE)


############
### "Auto" targets - main() files and unit tests

# Add "auto" executables
function(GT_AUTO_EXES)
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        foreach(src ${srcs})
            gt_get_stem(exe ${src})
            message(STATUS "[gt.cmake] Adding auto exe \"${exe}\" from ${src}")
            add_executable(${exe} ${src})
            if(TARGET ${PROJECT_NAME}-shared)
                target_link_libraries(${exe} ${PROJECT_NAME}-shared)
            elseif(TARGET ${PROJECT_NAME}-static)
                target_link_libraries(${exe} ${PROJECT_NAME}-static)
            else()
                message(FATAL_ERROR "[gt.cmake] No common source files specified yet!")
            endif()
        endforeach(src ${srcs})
    endforeach(srcpat ${ARGN})
endfunction(GT_AUTO_EXES)

# Add "auto" executables and install them
function(GT_AUTO_INSTALLED_EXES)
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        foreach(src ${srcs})
            gt_get_stem(exe ${src})
            message(STATUS "[gt.cmake] Adding auto installed exe \"${exe}\" from ${src}")
            add_executable(${exe} ${src})
            if(TARGET ${PROJECT_NAME}-shared)
                target_link_libraries(${exe} ${PROJECT_NAME}-shared)
            elseif(TARGET ${PROJECT_NAME}-static)
                target_link_libraries(${exe} ${PROJECT_NAME}-static)
            else()
                message(FATAL_ERROR "[gt.cmake] No common source files specified yet!")
            endif()
            install(TARGETS ${exe} RUNTIME DESTINATION "bin")
        endforeach(src ${srcs})
    endforeach(srcpat ${ARGN})
endfunction(GT_AUTO_INSTALLED_EXES)


# Add "auto" unit tests
function(GT_AUTO_TESTS)
    # Enable testing
    if(NOT gt_testing_on)
        message(STATUS "[gt.cmake] Enabling unit testing")
        include_directories("${GT_CPPUNITLITE_INCLUDE_DIR}")
        link_directories("${GT_CPPUNITLITE_LIB_DIR}")
        set(gt_testing_on 1 PARENT_SCOPE)
    endif(NOT gt_testing_on)
    
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        foreach(src ${srcs})
            gt_get_stem(exe ${src})
            message(STATUS "[gt.cmake] Adding test \"${exe}\" from ${src}")
            add_executable(${exe} ${src})
            target_link_libraries(${exe} "CppUnitLite")
            if(TARGET ${PROJECT_NAME}-shared)
                target_link_libraries(${exe} ${PROJECT_NAME}-shared)
            elseif(TARGET ${PROJECT_NAME}-static)
                target_link_libraries(${exe} ${PROJECT_NAME}-static)
            else()
                message(FATAL_ERROR "[gt.cmake] No common source files specified yet!")
            endif()
            add_test(${exe} ${exe})
        endforeach(src ${srcs})
    endforeach(srcpat ${ARGN})
endfunction(GT_AUTO_TESTS)


function(GT_GET_STEM outstem path)
    gt_get_pathstem(outpath stem "${path}")
    set(${outstem} "${stem}" PARENT_SCOPE)
    #message(STATUS "Got ${stem} for stem from child, setting into variable ${outstem}")
endfunction(GT_GET_STEM)

# Get the target name from a source file path
function(GT_GET_PATHSTEM outpath outstem path)
    #message(STATUS "Looking for stem of ${path}")
    # Get the file name from the path
    file(TO_CMAKE_PATH "${path}" stems)
    string(REPLACE "/" ";" stems "${path}")
    list(GET stems -1 stem)
    #message(STATUS "Got ${stem} for filename")
    list(REMOVE_AT stems -1)
    string(REPLACE ";" "/" stems "${stems}")
    file(TO_NATIVE_PATH "${stems}" stems)
    set(${outpath} "${stems}" PARENT_SCOPE)
    #message(STATUS "Got ${stems} for path")
    # Remove extension
    string(REPLACE "." ";" stem "${stem}")
    list(LENGTH stem llen)
    if(${llen} GREATER 1)
        list(REMOVE_AT stem -1)
    else(${llen} GREATER 1)
        list(APPEND stem "out")
    endif(${llen} GREATER 1)
    string(REPLACE ";" "." stem "${stem}")
    set(${outstem} "${stem}" PARENT_SCOPE)
    #message(STATUS "Got ${stem} for stem (setting into variable ${outstem})")
endfunction(GT_GET_PATHSTEM)


cmake_policy(POP)

