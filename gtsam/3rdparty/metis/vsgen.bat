MKDIR build\windows
CD build\windows
cmake -DCMAKE_CONFIGURATION_TYPES="Release" ..\.. %*
ECHO VS files have been generated in build\windows
CD ..\..\
