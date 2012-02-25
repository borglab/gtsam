# Macros for using wrap functionality
macro(find_mexextension)
    ## Determine the mex extension
    # Apple Macintosh (64-bit) mexmaci64
    # Linux (32-bit) mexglx
    # Linux (64-bit) mexa64
    # Microsoft Windows (32-bit) mexw32
    # Windows (64-bit) mexw64
    
    # only support 64-bit apple
    if(CMAKE_HOST_APPLE)
        set(GTSAM_MEX_BIN_EXTENSION_default mexmaci64)
    endif(CMAKE_HOST_APPLE)
    
    if(NOT CMAKE_HOST_APPLE)
        # check 64 bit
        if( ${CMAKE_SIZEOF_VOID_P} EQUAL 4 )
          set( HAVE_64_BIT 0 )
        endif( ${CMAKE_SIZEOF_VOID_P} EQUAL 4 )
        
        if( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
          set( HAVE_64_BIT 1 )
        endif( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
        
        # Check for linux machines
        if (CMAKE_HOST_UNIX)
            if (HAVE_64_BIT)
                set(GTSAM_MEX_BIN_EXTENSION_default mexa64)
            else (HAVE_64_BIT)
                set(GTSAM_MEX_BIN_EXTENSION_default mexglx)
            endif (HAVE_64_BIT)
        endif(CMAKE_HOST_UNIX)
        
        # Check for windows machines
        if (CMAKE_HOST_WIN32)
            if (HAVE_64_BIT)
                set(GTSAM_MEX_BIN_EXTENSION_default mexw64)
            else (HAVE_64_BIT)
                set(GTSAM_MEX_BIN_EXTENSION_default mexw32)
            endif (HAVE_64_BIT)
        endif(CMAKE_HOST_WIN32)
    endif(NOT CMAKE_HOST_APPLE)
    
    # Allow for setting mex extension manually
    set(GTSAM_MEX_BIN_EXTENSION ${GTSAM_MEX_BIN_EXTENSION_default} CACHE DOCSTRING "Extension for matlab mex files")
    message(STATUS "Detected Matlab mex extension: ${GTSAM_MEX_BIN_EXTENSION_default}")
    message(STATUS "Current Matlab mex extension: ${GTSAM_MEX_BIN_EXTENSION}")
endmacro(find_mexextension)