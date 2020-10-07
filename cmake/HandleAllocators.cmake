# Build list of possible allocators
set(possible_allocators "")
if(GTSAM_USE_TBB)
    list(APPEND possible_allocators TBB)
    set(preferred_allocator TBB)
else()
    list(APPEND possible_allocators BoostPool STL)
    set(preferred_allocator STL)
endif()
if(GOOGLE_PERFTOOLS_FOUND)
    list(APPEND possible_allocators tcmalloc)
endif()

# Check if current allocator choice is valid and set cache option
list(FIND possible_allocators "${GTSAM_DEFAULT_ALLOCATOR}" allocator_valid)
if(allocator_valid EQUAL -1)
    set(GTSAM_DEFAULT_ALLOCATOR ${preferred_allocator} CACHE STRING "Default allocator" FORCE)
else()
    set(GTSAM_DEFAULT_ALLOCATOR ${preferred_allocator} CACHE STRING "Default allocator")
endif()
set_property(CACHE GTSAM_DEFAULT_ALLOCATOR PROPERTY STRINGS ${possible_allocators})
mark_as_advanced(GTSAM_DEFAULT_ALLOCATOR)

# Define compile flags depending on allocator
if("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "BoostPool")
    set(GTSAM_ALLOCATOR_BOOSTPOOL 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "STL")
    set(GTSAM_ALLOCATOR_STL 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "TBB")
    set(GTSAM_ALLOCATOR_TBB 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "tcmalloc")
    set(GTSAM_ALLOCATOR_STL 1) # tcmalloc replaces malloc, so to use it we use the STL allocator
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES "tcmalloc")
endif()
