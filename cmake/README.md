# GTSAMCMakeTools

This is the collection of GTSAM CMake tools that may be useful in external projects.  The way to use this collection is by first making a find_package call:

    find_package(GTSAMCMakeTools)

which will add a directory containing the GTSAM CMake tools to the CMAKE_MODULE_PATH variable.  After that, you may include the files you would like to use.  These files and the functions they define are explained below.

## GtsamBuildTypes

    include(GtsamBuildTypes)

Including this file immediately sets up the following build types and a drop-down list in cmake-gui:

*   `Debug`
*   `Release`
*   `RelWithDebInfo`
*   `Profiling`: All optimizations enabled and minimal debug symbols
*   `Timing`: Defines the symbol GTSAM_ENABLE_TIMING for using GTSAM timing instrumentation

It also configures several minor details, as follows:

*   The compile flag `-ftemplate-depth=1024` is set for newer versions of Clang to handle complex templates.
*   On Windows, executable and dll output paths are set to `${CMAKE_BINARY_DIR}/bin` and import library output to `${CMAKE_BINARY_DIR}/lib`.

It defines the following functions:

*   `gtsam_assign_source_folders( [files] )` Organizes files in the IDE into folders to reflect the actual directory structure of those files.  Folders will be determined relative to the current source folder when this function is called.
*   `gtsam_assign_all_source_folders()` Calls `gtsam_assign_source_folders` on all cpp, c, and h files recursively in the current source folder.

## GtsamTesting

    include(GtsamTesting)

Defines two useful functions for creating CTest unit tests.  Also immediately creates a `check` target that builds and runs all unit tests.

*   `gtsamAddTestsGlob(groupName globPatterns excludedFiles linkLibraries)` Add a group of unit tests.  A list of unit test .cpp files or glob patterns specifies the tests to create.  Tests are assigned into a group name so they can easily by run independently with a make target.  Running 'make check' builds and runs all tests.

    Usage example:
    
        gtsamAddTestsGlob(basic "test*.cpp" "testBroken.cpp" "gtsam;GeographicLib")

    Arguments:
    
        groupName:     A name that will allow this group of tests to be run independently, e.g.
                       'basic' causes a 'check.basic' target to be created to run this test
                       group.
        globPatterns:  The list of files or glob patterns from which to create unit tests, with
                       one test created for each cpp file.  e.g. "test*.cpp", or
                       "testA*.cpp;testB*.cpp;testOneThing.cpp".
        excludedFiles: A list of files or globs to exclude, e.g. "testC*.cpp;testBroken.cpp".
                       Pass an empty string "" if nothing needs to be excluded.
        linkLibraries: The list of libraries to link to in addition to CppUnitLite.
        
*   `gtsamAddExamplesGlob(globPatterns excludedFiles linkLibraries)` Add scripts that will serve as examples of how to use the library.  A list of files or glob patterns is specified, and one executable will be created for each matching .cpp file.  These executables will not be installed.  They are build with 'make all' if GTSAM_BUILD_EXAMPLES_ALWAYS is enabled.  They may also be built with 'make examples'.

    Usage example:

        gtsamAddExamplesGlob("*.cpp" "BrokenExample.cpp" "gtsam;GeographicLib")

    Arguments:

        globPatterns:  The list of files or glob patterns from which to create unit tests, with
                       one test created for each cpp file.  e.g. "*.cpp", or
                       "A*.cpp;B*.cpp;MyExample.cpp".
        excludedFiles: A list of files or globs to exclude, e.g. "C*.cpp;BrokenExample.cpp".  Pass
                       an empty string "" if nothing needs to be excluded.
        linkLibraries: The list of libraries to link to.

## GtsamMakeConfigFile

    include(GtsamMakeConfigFile)
     
Defines a function for generating a config file so your project may be found with the CMake `find_package` function.  TODO: Write documentation.