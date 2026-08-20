option(GTSAM_ENABLE_CUDA "Enable experimental CUDA acceleration support" OFF)
option(GTSAM_ENABLE_CUDSS "Enable experimental cuDSS linear solver support" OFF)

if(GTSAM_ENABLE_CUDSS AND NOT GTSAM_ENABLE_CUDA)
  message(FATAL_ERROR "GTSAM_ENABLE_CUDSS=ON requires GTSAM_ENABLE_CUDA=ON")
endif()

if(GTSAM_ENABLE_CUDA AND CMAKE_VERSION VERSION_LESS "3.17")
  message(FATAL_ERROR "GTSAM_ENABLE_CUDA requires CMake 3.17 or newer")
endif()

# The CUDA kernels accumulate into double-precision buffers with atomicAdd,
# which requires compute capability 6.0 (Pascal, 2016) or newer.
set(GTSAM_CUDA_MIN_ARCHITECTURE 60)

# CudssSpdSolver.cpp uses CUDSS_R_64F, CUDSS_R_32I, cudssReorderingAlg_t,
# CUDSS_REORDERING_ALG_NONE, and CUDSS_STATUS_IR_FAILED, all of which arrived in
# cuDSS 0.8.0. Older cuDSS is rejected at configure time rather than as errors
# on the API it does not have.
set(GTSAM_CUDSS_MINIMUM_VERSION 0.8.0)

# Reports the compute capabilities of the installed GPUs, so that "native" can
# be checked against a candidate compiler before enable_language(CUDA) resolves
# it. Leaves the output empty when no driver is available to ask.
function(gtsam_cuda_detect_native_architectures out_var)
  find_program(GTSAM_NVIDIA_SMI nvidia-smi)
  set(${out_var} "" PARENT_SCOPE)
  if(NOT GTSAM_NVIDIA_SMI)
    return()
  endif()
  execute_process(
    COMMAND "${GTSAM_NVIDIA_SMI}" --query-gpu=compute_cap --format=csv,noheader
    OUTPUT_VARIABLE gtsam_smi_output
    RESULT_VARIABLE gtsam_smi_result
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(NOT gtsam_smi_result EQUAL 0)
    return()
  endif()
  string(REPLACE "\n" ";" gtsam_smi_lines "${gtsam_smi_output}")
  set(gtsam_detected "")
  foreach(gtsam_line IN LISTS gtsam_smi_lines)
    if(gtsam_line MATCHES "([0-9]+)\\.([0-9]+)")
      list(APPEND gtsam_detected "${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES gtsam_detected)
  set(${out_var} "${gtsam_detected}" PARENT_SCOPE)
endfunction()

# Compiles an empty kernel for each architecture to establish whether one nvcc
# can target them all. Toolkits drop old architectures and predate new ones, so
# the answer depends on the pair and is cheapest to obtain by asking. Sets
# out_var to OK, UNSUPPORTED_ARCH, or UNUSABLE; the last means the compiler
# failed even without an architecture flag, so nothing can be concluded about
# the architectures and the real error belongs to the build. On
# UNSUPPORTED_ARCH, out_arch names the first architecture that was refused.
function(gtsam_cuda_probe_compiler nvcc architectures out_var out_arch)
  set(gtsam_probe_source
      "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/gtsamCudaArchProbe.cu")
  file(WRITE "${gtsam_probe_source}" "__global__ void gtsamCudaArchProbe() {}\n")

  # Mirror the host compiler the build will hand to nvcc, so that an unusable
  # toolkit is recognized here rather than halfway through compiling.
  set(gtsam_probe_ccbin "")
  if(CMAKE_CUDA_HOST_COMPILER)
    set(gtsam_probe_ccbin -ccbin "${CMAKE_CUDA_HOST_COMPILER}")
  endif()

  execute_process(
    COMMAND "${nvcc}" ${gtsam_probe_ccbin} -ptx "${gtsam_probe_source}"
            -o "${gtsam_probe_source}.ptx"
    RESULT_VARIABLE gtsam_probe_result OUTPUT_QUIET ERROR_QUIET)
  if(NOT gtsam_probe_result EQUAL 0)
    set(${out_var} UNUSABLE PARENT_SCOPE)
    set(${out_arch} "" PARENT_SCOPE)
    return()
  endif()

  foreach(gtsam_arch IN LISTS architectures)
    execute_process(
      COMMAND "${nvcc}" ${gtsam_probe_ccbin} "-arch=sm_${gtsam_arch}" -ptx
              "${gtsam_probe_source}" -o "${gtsam_probe_source}.ptx"
      RESULT_VARIABLE gtsam_probe_result OUTPUT_QUIET ERROR_QUIET)
    if(NOT gtsam_probe_result EQUAL 0)
      set(${out_var} UNSUPPORTED_ARCH PARENT_SCOPE)
      set(${out_arch} "${gtsam_arch}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${out_var} OK PARENT_SCOPE)
  set(${out_arch} "" PARENT_SCOPE)
endfunction()

# Reports a toolkit version like "12.0.140" for use in diagnostics.
function(gtsam_cuda_compiler_version nvcc out_var)
  execute_process(COMMAND "${nvcc}" --version
                  OUTPUT_VARIABLE gtsam_nvcc_banner ERROR_QUIET)
  if(gtsam_nvcc_banner MATCHES "V([0-9]+\\.[0-9]+\\.[0-9]+)")
    set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
  else()
    set(${out_var} "unknown version" PARENT_SCOPE)
  endif()
endfunction()

if(GTSAM_ENABLE_CUDA)
  # Without this, CMP0104 leaves CMAKE_CUDA_ARCHITECTURES at nvcc's own default,
  # which is 52 through CUDA 12 and therefore cannot compile the kernels. Choose
  # a default before enable_language(CUDA) so it is not overwritten. "native"
  # builds only for the GPUs present, so name the architectures explicitly when
  # producing binaries for other machines.
  gtsam_cuda_detect_native_architectures(gtsam_cuda_detected)

  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
      set(CMAKE_CUDA_ARCHITECTURES "native"
          CACHE STRING "CUDA architectures GTSAM generates code for")
    else()
      # No "native" before 3.24, so name what was detected. A fixed default
      # cannot work: the oldest architecture the kernels support is also one
      # that CUDA 13 has already dropped.
      set(CMAKE_CUDA_ARCHITECTURES "${gtsam_cuda_detected}"
          CACHE STRING "CUDA architectures GTSAM generates code for")
    endif()
  endif()

  # The architectures as plain numbers, which is what nvcc has to be asked
  # about. "native" is CMake's word, not nvcc's, so use what was detected;
  # "all" and "all-major" are known only to CMake and stay unexpanded, in
  # which case the checks below are skipped rather than guessed at.
  set(gtsam_cuda_numeric_architectures "")
  if(CMAKE_CUDA_ARCHITECTURES STREQUAL "native" OR
     NOT CMAKE_CUDA_ARCHITECTURES)
    set(gtsam_cuda_numeric_architectures "${gtsam_cuda_detected}")
    if(NOT gtsam_cuda_numeric_architectures)
      # Asking nvcc for "native" without a GPU to read is not an error to it:
      # it warns and compiles for its own default architecture, which is 52
      # through CUDA 12 and so fails inside the kernels much later. Ask for the
      # architectures instead of building something that cannot run.
      message(FATAL_ERROR
        "GTSAM_ENABLE_CUDA builds for the GPUs of the configuring machine by "
        "default, but the compute capability of no installed GPU could be read. "
        "Name the architectures to build for, for example "
        "-DCMAKE_CUDA_ARCHITECTURES=80;120, or configure with "
        "GTSAM_ENABLE_CUDA=OFF.")
    endif()
  else()
    foreach(gtsam_cuda_arch IN LISTS CMAKE_CUDA_ARCHITECTURES)
      string(REGEX REPLACE "-(real|virtual)$" "" gtsam_cuda_arch
             "${gtsam_cuda_arch}")
      if(gtsam_cuda_arch MATCHES "^[0-9]+[a-z]*$")
        list(APPEND gtsam_cuda_numeric_architectures "${gtsam_cuda_arch}")
      endif()
    endforeach()
  endif()

  # Choose the CUDA compiler when the user has not. CMake searches PATH only,
  # which on a machine with several toolkits installed can find an nvcc too old
  # for the GPU that is present -- CUDA 12.0 cannot target a Blackwell card even
  # though the driver supports it. Prefer the locations that name a toolkit
  # explicitly, then the conventional symlink, then PATH, then the newest
  # versioned install, and take the first one that compiles for the requested
  # architectures.
  set(gtsam_cuda_rejected "")
  if(NOT CMAKE_CUDA_COMPILER AND NOT DEFINED ENV{CUDACXX})
    # Roots the user pointed at, which are a deliberate choice and so come first.
    set(gtsam_cuda_candidates "")
    foreach(gtsam_cuda_root IN ITEMS
        "${CUDAToolkit_ROOT}" "$ENV{CUDAToolkit_ROOT}" "$ENV{CUDA_HOME}"
        "$ENV{CUDA_PATH}")
      if(gtsam_cuda_root AND EXISTS "${gtsam_cuda_root}/bin/nvcc")
        list(APPEND gtsam_cuda_candidates "${gtsam_cuda_root}/bin/nvcc")
      endif()
    endforeach()
    set(gtsam_cuda_hinted "${gtsam_cuda_candidates}")

    if(EXISTS "/usr/local/cuda/bin/nvcc")
      list(APPEND gtsam_cuda_candidates "/usr/local/cuda/bin/nvcc")
    endif()
    find_program(GTSAM_CUDA_PATH_NVCC nvcc)
    if(GTSAM_CUDA_PATH_NVCC)
      list(APPEND gtsam_cuda_candidates "${GTSAM_CUDA_PATH_NVCC}")
    endif()

    # Side-by-side installs, newest first, selected by repeated maximum because
    # list(SORT COMPARE NATURAL) needs a newer CMake than CUDA support does.
    set(gtsam_cuda_pending "")
    file(GLOB gtsam_cuda_versioned LIST_DIRECTORIES true "/usr/local/cuda-*")
    foreach(gtsam_cuda_root IN LISTS gtsam_cuda_versioned)
      if(EXISTS "${gtsam_cuda_root}/bin/nvcc" AND
         gtsam_cuda_root MATCHES "cuda-([0-9]+(\\.[0-9]+)*)$")
        list(APPEND gtsam_cuda_pending
             "${CMAKE_MATCH_1}|${gtsam_cuda_root}/bin/nvcc")
      endif()
    endforeach()
    while(gtsam_cuda_pending)
      set(gtsam_cuda_newest "")
      set(gtsam_cuda_newest_version "0")
      foreach(gtsam_cuda_entry IN LISTS gtsam_cuda_pending)
        string(REGEX REPLACE "\\|.*$" "" gtsam_cuda_entry_version
               "${gtsam_cuda_entry}")
        if(gtsam_cuda_entry_version VERSION_GREATER gtsam_cuda_newest_version)
          set(gtsam_cuda_newest_version "${gtsam_cuda_entry_version}")
          set(gtsam_cuda_newest "${gtsam_cuda_entry}")
        endif()
      endforeach()
      string(REGEX REPLACE "^[^|]*\\|" "" gtsam_cuda_newest_path
             "${gtsam_cuda_newest}")
      list(APPEND gtsam_cuda_candidates "${gtsam_cuda_newest_path}")
      list(REMOVE_ITEM gtsam_cuda_pending "${gtsam_cuda_newest}")
    endwhile()
    list(REMOVE_DUPLICATES gtsam_cuda_candidates)

    if(NOT gtsam_cuda_numeric_architectures)
      # Nothing to test against, so only honour an explicitly named root and
      # otherwise leave the choice to CMake.
      if(gtsam_cuda_hinted)
        list(GET gtsam_cuda_hinted 0 gtsam_cuda_chosen)
      endif()
    else()
      foreach(gtsam_cuda_candidate IN LISTS gtsam_cuda_candidates)
        gtsam_cuda_probe_compiler("${gtsam_cuda_candidate}"
                                  "${gtsam_cuda_numeric_architectures}"
                                  gtsam_cuda_verdict gtsam_cuda_bad_arch)
        if(gtsam_cuda_verdict STREQUAL "OK")
          set(gtsam_cuda_chosen "${gtsam_cuda_candidate}")
          break()
        endif()
        gtsam_cuda_compiler_version("${gtsam_cuda_candidate}" gtsam_cuda_version)
        # Keyed by version so that the same toolkit reached through a symlink
        # and its versioned directory is only reported once.
        set(gtsam_cuda_because "")
        if(gtsam_cuda_bad_arch)
          set(gtsam_cuda_because ", no ${gtsam_cuda_bad_arch}")
        endif()
        list(APPEND gtsam_cuda_rejected
             "${gtsam_cuda_version}|${gtsam_cuda_candidate} (${gtsam_cuda_version}${gtsam_cuda_because})")
      endforeach()
    endif()

    if(gtsam_cuda_chosen)
      set(CMAKE_CUDA_COMPILER "${gtsam_cuda_chosen}"
          CACHE FILEPATH "CUDA compiler GTSAM builds the kernels with")
      # Keep the toolkit libraries and headers with the compiler that was
      # picked, rather than letting the search find a different install.
      if(NOT CUDAToolkit_ROOT)
        get_filename_component(gtsam_cuda_chosen_root "${gtsam_cuda_chosen}"
                               DIRECTORY)
        get_filename_component(gtsam_cuda_chosen_root
                               "${gtsam_cuda_chosen_root}" DIRECTORY)
        set(CUDAToolkit_ROOT "${gtsam_cuda_chosen_root}"
            CACHE PATH "CUDA toolkit GTSAM links against")
      endif()
      message(STATUS "GTSAM CUDA compiler: ${gtsam_cuda_chosen}")
    endif()
  endif()

  # Catch an unusable architecture, including one inherited from a build
  # directory configured before the default above existed.
  foreach(gtsam_cuda_arch IN LISTS CMAKE_CUDA_ARCHITECTURES
                                   gtsam_cuda_numeric_architectures)
    if(gtsam_cuda_arch MATCHES "^([0-9]+)")
      if(CMAKE_MATCH_1 LESS GTSAM_CUDA_MIN_ARCHITECTURE)
        set(gtsam_cuda_resolved "")
        if(NOT CMAKE_CUDA_ARCHITECTURES STREQUAL
           "${gtsam_cuda_numeric_architectures}")
          string(REPLACE ";" " " gtsam_cuda_resolved
                 "${gtsam_cuda_numeric_architectures}")
          set(gtsam_cuda_resolved " (${gtsam_cuda_resolved})")
        endif()
        message(FATAL_ERROR
          "GTSAM_ENABLE_CUDA requires compute capability "
          "${GTSAM_CUDA_MIN_ARCHITECTURE} or newer, but CMAKE_CUDA_ARCHITECTURES "
          "is '${CMAKE_CUDA_ARCHITECTURES}'${gtsam_cuda_resolved}. The kernels "
          "use double-precision atomicAdd, which architecture "
          "${CMAKE_MATCH_1} does not provide. Build for a newer GPU, or "
          "configure with GTSAM_ENABLE_CUDA=OFF.")
      endif()
    endif()
  endforeach()

  # Report a compiler that cannot target the requested architectures now, rather
  # than after a long build ends in "nvcc fatal: Unsupported gpu architecture".
  # This runs before enable_language(CUDA), whose own compiler test would
  # otherwise fail first with no indication of which knob to turn. When the
  # search above chose nothing, resolve the compiler the way CMake will.
  set(gtsam_cuda_compiler "${CMAKE_CUDA_COMPILER}")
  if(NOT gtsam_cuda_compiler)
    set(gtsam_cuda_compiler "$ENV{CUDACXX}")
  endif()
  if(NOT gtsam_cuda_compiler)
    find_program(GTSAM_CUDA_PATH_NVCC nvcc)
    set(gtsam_cuda_compiler "${GTSAM_CUDA_PATH_NVCC}")
  endif()

  if(gtsam_cuda_numeric_architectures AND EXISTS "${gtsam_cuda_compiler}")
    gtsam_cuda_probe_compiler("${gtsam_cuda_compiler}"
                              "${gtsam_cuda_numeric_architectures}"
                              gtsam_cuda_verdict gtsam_cuda_bad_arch)
    if(gtsam_cuda_verdict STREQUAL "UNSUPPORTED_ARCH")
      gtsam_cuda_compiler_version("${gtsam_cuda_compiler}" gtsam_cuda_version)
      string(REPLACE ";" " " gtsam_cuda_requested
             "${gtsam_cuda_numeric_architectures}")
      # Name the other toolkits that were searched, skipping any that is the
      # same version as the one being reported, which would add nothing.
      set(gtsam_cuda_others "")
      set(gtsam_cuda_seen_versions "${gtsam_cuda_version}")
      foreach(gtsam_cuda_entry IN LISTS gtsam_cuda_rejected)
        string(REGEX REPLACE "\\|.*$" "" gtsam_cuda_entry_version
               "${gtsam_cuda_entry}")
        if(NOT gtsam_cuda_entry_version IN_LIST gtsam_cuda_seen_versions)
          list(APPEND gtsam_cuda_seen_versions "${gtsam_cuda_entry_version}")
          string(REGEX REPLACE "^[^|]*\\|" "" gtsam_cuda_entry
                 "${gtsam_cuda_entry}")
          list(APPEND gtsam_cuda_others "${gtsam_cuda_entry}")
        endif()
      endforeach()
      set(gtsam_cuda_report "")
      if(gtsam_cuda_others)
        string(REPLACE ";" ", " gtsam_cuda_others "${gtsam_cuda_others}")
        set(gtsam_cuda_report " Also tried: ${gtsam_cuda_others}.")
      endif()
      message(FATAL_ERROR
        "CUDA compiler ${gtsam_cuda_compiler} (${gtsam_cuda_version}) cannot "
        "target architecture ${gtsam_cuda_bad_arch}, requested as "
        "'${gtsam_cuda_requested}'. Select a compatible compiler using CUDACXX "
        "or -DCMAKE_CUDA_COMPILER=/path/to/nvcc, or request architectures it "
        "supports with -DCMAKE_CUDA_ARCHITECTURES.${gtsam_cuda_report}")
    endif()
  endif()

  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  message(STATUS "GTSAM CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")

  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_EXTENSIONS OFF)

  # Eigen's device code calls constexpr host functions, which nvcc allows only
  # with this flag and otherwise reports once per translation unit.
  string(APPEND CMAKE_CUDA_FLAGS " --expt-relaxed-constexpr")

  # nvcc's frontend also reports two diagnostics in long-standing GTSAM headers
  # that neither GCC nor Clang report: #611-D for the partially overridden
  # whitenInPlace overload set in NoiseModel.h, and #997-D for the Factor::error
  # overload hidden in BatchFactor.h. Both repeat in every CUDA translation unit
  # and drown out warnings about the kernels themselves. Silence them here;
  # changing those overload sets belongs with those classes rather than with
  # CUDA support.
  string(APPEND CMAKE_CUDA_FLAGS " -diag-suppress=611 -diag-suppress=997")

  list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC GTSAM_ENABLE_CUDA=1)

  if(GTSAM_ENABLE_CUDSS)
    # cuDSS is not part of the CUDA toolkit; it is a separate NVIDIA download.
    # docs/CUDA_LINEAR_SOLVERS.md records where to get it. Keep this search in
    # step with the copy in cmake/Config.cmake.in, which is what downstream
    # projects consuming an installed GTSAM run.
    if(NOT TARGET cudss::cudss)
      find_path(CUDSS_INCLUDE_DIR cudss.h
        HINTS
          ${CUDSS_ROOT}/include
          $ENV{CUDSS_ROOT}/include
          ${CUDAToolkit_INCLUDE_DIRS}
          $ENV{CUDA_HOME}/include
          $ENV{CUDA_PATH}/include
          $ENV{CONDA_PREFIX}/include
        PATH_SUFFIXES
          libcudss/13
          libcudss/12
      )

      find_library(CUDSS_LIBRARY cudss
        HINTS
          ${CUDSS_ROOT}/lib64
          ${CUDSS_ROOT}/lib
          $ENV{CUDSS_ROOT}/lib64
          $ENV{CUDSS_ROOT}/lib
          ${CUDAToolkit_LIBRARY_DIR}
          ${CUDAToolkit_LIBRARY_ROOT}
          $ENV{CUDA_HOME}/lib64
          $ENV{CUDA_PATH}/lib64
          $ENV{CONDA_PREFIX}/lib
        PATH_SUFFIXES
          libcudss/13
          libcudss/12
      )

      if(NOT EXISTS "${CUDSS_INCLUDE_DIR}/cudss.h" OR NOT CUDSS_LIBRARY)
        message(FATAL_ERROR
          "GTSAM_ENABLE_CUDSS=ON but cuDSS was not found (cudss.h: "
          "${CUDSS_INCLUDE_DIR}, libcudss: ${CUDSS_LIBRARY}).\n"
          "cuDSS is a separate NVIDIA download rather than part of the CUDA "
          "toolkit; see docs/CUDA_LINEAR_SOLVERS.md. Set CUDSS_ROOT to an "
          "existing install, or build with -DGTSAM_ENABLE_CUDSS=OFF to use the "
          "PCG and dense Cholesky backends only.")
      endif()

      # Read the version from the header rather than from the CMake config
      # package cuDSS ships: the Debian packages install headers outside the
      # prefix that config computes from its own location, so
      # find_package(cudss CONFIG) hard-errors on them. The header is the one
      # file every packaging puts in the same place.
      set(_cudss_version_parts "")
      file(STRINGS "${CUDSS_INCLUDE_DIR}/cudss.h" _cudss_version_lines
        REGEX "^#define CUDSS_VERSION_(MAJOR|MINOR|PATCH)[ \t]+[0-9]+")
      foreach(_component MAJOR MINOR PATCH)
        foreach(_line IN LISTS _cudss_version_lines)
          if(_line MATCHES "CUDSS_VERSION_${_component}[ \t]+([0-9]+)")
            list(APPEND _cudss_version_parts "${CMAKE_MATCH_1}")
          endif()
        endforeach()
      endforeach()
      string(REPLACE ";" "." CUDSS_VERSION "${_cudss_version_parts}")

      if(NOT CUDSS_VERSION)
        message(FATAL_ERROR
          "Could not read CUDSS_VERSION_MAJOR, CUDSS_VERSION_MINOR, and "
          "CUDSS_VERSION_PATCH from ${CUDSS_INCLUDE_DIR}/cudss.h, so cuDSS "
          "cannot be checked against the required "
          "${GTSAM_CUDSS_MINIMUM_VERSION}.")
      endif()

      if(CUDSS_VERSION VERSION_LESS GTSAM_CUDSS_MINIMUM_VERSION)
        message(FATAL_ERROR
          "cuDSS ${CUDSS_VERSION} found at ${CUDSS_INCLUDE_DIR}, but GTSAM "
          "requires ${GTSAM_CUDSS_MINIMUM_VERSION} or newer.\n"
          "Set CUDSS_ROOT to a newer install, or build with "
          "-DGTSAM_ENABLE_CUDSS=OFF to use the PCG and dense Cholesky backends "
          "only. An active conda or virtual environment can supply an older "
          "cuDSS than the system one.")
      endif()

      add_library(cudss::cudss UNKNOWN IMPORTED)
      set_target_properties(cudss::cudss PROPERTIES
        IMPORTED_LOCATION "${CUDSS_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CUDSS_INCLUDE_DIR};${CUDAToolkit_INCLUDE_DIRS}")
      message(STATUS "GTSAM cuDSS: ${CUDSS_VERSION} (${CUDSS_LIBRARY})")
    endif()
    list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC GTSAM_ENABLE_CUDSS=1)
  endif()
endif()
