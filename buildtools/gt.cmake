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
set(CMAKE_INSTALL_PREFIX "$ENV{HOME}" CACHE PATH "Installation prefix")
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type, Debug or Release")

# Add borg libraries and includes
include_directories("${CMAKE_INSTALL_PREFIX}/include")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")

# Path to CppUnitLite
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
message(STATUS "Build tools dir ${GT_BUILDTOOLS}")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${GT_BUILDTOOLS}")
function(GT_USE_BOOST)
    find_package(Boost REQUIRED)
    link_directories(${Boost_LIBRARY_DIRS})
    include_directories(${Boost_INCLUDE_DIRS})
    link_libraries(${Boost_LIBRARIES})
endfunction(GT_USE_BOOST)
function(GT_USE_QT4)
    find_package(Qt4 COMPONENTS QtCore QtGui ${ARGN} REQUIRED)
    include(${QT_USE_FILE})
    link_libraries(${QT_LIBRARIES})
endfunction(GT_USE_QT4)
function(GT_USE_OPENCV)
    find_package(OpenCV REQUIRED)
    link_directories(${OpenCV_LINK_DIRECTORIES})
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBRARIES)
endfunction(GT_USE_OPENCV)
function(GT_USE_GTSAM)
    link_libraries("gtsam")
endfunction(GT_USE_GTSAM)
function(GT_USE_EASY2D)
    link_libraries("easy2d")
endfunction(GT_USE_EASY2D)


############
### Main target functions, calling one of these will replace the previous main target.

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
    set(GT_COMMON_SOURCES "")
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        set(GT_COMMON_SOURCES ${GT_COMMON_SOURCES} ${srcs})
        #message(STATUS "Adding sources: " ${srcs})
    endforeach(srcpat)
    list(REMOVE_ITEM GT_COMMON_SOURCES ${GT_EXCLUDE_SOURCES})
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
endfunction(GT_MAIN_SOURCES)

# Add headers to be installed
function(GT_INSTALL_HEADERS)
    foreach(srcpat ${ARGN})
        # Get the sources matching the specified pattern
        file(GLOB srcs RELATIVE "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${srcpat}")
        install(FILES ${srcs} DESTINATION "include/${PROJECT_NAME}")
    endforeach(srcpat)
endfunction(GT_INSTALL_HEADERS)

# Define the main shared+static library from the given sources
function(GT_MAIN_LIB)
    gt_main_sources(${ARGN})
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
    message(STATUS "[gt.cmake] Adding main lib \"${PROJECT_NAME}\" with sources ${GT_COMMON_SOURCES}")
    add_library(${PROJECT_NAME} SHARED ${GT_COMMON_SOURCES})
    add_library(${PROJECT_NAME}-static STATIC ${GT_COMMON_SOURCES})
    SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    SET_TARGET_PROPERTIES(${PROJECT_NAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}-static LIBRARY DESTINATION "lib" ARCHIVE DESTINATION "lib")
endfunction(GT_MAIN_LIB)

# Define the main static library from the given sources
function(GT_MAIN_STLIB)
    gt_main_sources(${ARGN})
    set(GT_COMMON_SOURCES "${GT_COMMON_SOURCES}" PARENT_SCOPE)
    message(STATUS "[gt.cmake] Adding main stlib \"${PROJECT_NAME}\" with sources ${GT_COMMON_SOURCES}")
    add_library(${PROJECT_NAME} STATIC ${GT_COMMON_SOURCES})
    install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION "lib")
endfunction(GT_MAIN_STLIB)

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
            add_executable(${exe} ${src} ${GT_COMMON_SOURCES})
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
            add_executable(${exe} ${src} ${GT_COMMON_SOURCES})
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
            add_executable(${exe} ${src} ${GT_COMMON_SOURCES})
            target_link_libraries(${exe} "CppUnitLite")
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

