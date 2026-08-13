###############################################################################
# Find MKL
find_package(MKL)

if(MKL_FOUND AND GTSAM_WITH_EIGEN_MKL)
    set(GTSAM_USE_EIGEN_MKL 1) # This will go into config.h
    set(EIGEN_USE_MKL_ALL 1) # This will go into config.h - it makes Eigen use MKL
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES ${MKL_LIBRARIES})

    # --no-as-needed is required with gcc according to the MKL link advisor
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
    endif()
else()
    set(GTSAM_USE_EIGEN_MKL 0)
    set(EIGEN_USE_MKL_ALL 0)
endif()

if(WIN32 AND GTSAM_USE_EIGEN_MKL AND DEFINED VCPKG_INSTALLED_DIR)
    get_target_property(MKL_TARGETS "MKL::MKL" INTERFACE_LINK_LIBRARIES)
    set(RUNTIME_DLL_DIRS "")
    foreach(MKL_TARGET ${MKL_TARGETS})
        if(TARGET ${MKL_TARGET})
            get_target_property(MKL_DLL "${MKL_TARGET}" IMPORTED_LOCATION)
            if(MKL_DLL)
                cmake_path(GET MKL_DLL PARENT_PATH MKL_DLL_DIR)
                list (APPEND RUNTIME_DLL_DIRS "${MKL_DLL_DIR}/mkl_*.dll")
            endif()
        endif()
    endforeach()
    list(REMOVE_DUPLICATES RUNTIME_DLL_DIRS)
    file(GLOB MKL_DLLS CONFIGURE_DEPENDS ${RUNTIME_DLL_DIRS})
    list(REMOVE_DUPLICATES MKL_DLLS)
    add_custom_target(copy_mkl_dlls
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MKL_DLLS}"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
        COMMAND_EXPAND_LISTS
        VERBATIM
    )
    add_dependencies(check copy_mkl_dlls)
endif()
