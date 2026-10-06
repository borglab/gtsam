#!/usr/bin/env bash
# Cache objects outside build/: unix.sh intentionally deletes that directory.
set -euo pipefail

case "$1" in
  setup)
    probe=$(mktemp -d "$RUNNER_TEMP/compiler-probe.XXXXXX")
    cat > "$probe/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(CacheToolchain LANGUAGES C CXX)
if(EXISTS "/etc/os-release")
  file(SHA256 "/etc/os-release" os_release)
endif()
set(identity "${os_release};${CMAKE_VERSION};${CMAKE_SYSTEM_NAME};${CMAKE_SYSTEM_PROCESSOR};${CMAKE_SIZEOF_VOID_P}")
foreach(language C CXX)
  file(SHA256 "${CMAKE_${language}_COMPILER}" compiler_hash)
  string(APPEND identity ";${CMAKE_${language}_COMPILER};${compiler_hash};${CMAKE_${language}_COMPILER_ID};${CMAKE_${language}_COMPILER_VERSION};${CMAKE_${language}_COMPILER_TARGET};${CMAKE_${language}_COMPILER_ARCHITECTURE_ID}")
endforeach()
if(APPLE)
  execute_process(COMMAND xcrun --show-sdk-path OUTPUT_VARIABLE sdk_path)
  execute_process(COMMAND xcrun --show-sdk-version OUTPUT_VARIABLE sdk_version)
  execute_process(COMMAND xcodebuild -version OUTPUT_VARIABLE xcode_version)
  string(APPEND identity ";${sdk_path};${sdk_version};${xcode_version}")
endif()
# Include the selected SDK/toolset, not just the runner label or requested version.
string(APPEND identity ";${CMAKE_OSX_SYSROOT};$ENV{SDKROOT};$ENV{VCToolsVersion};$ENV{WindowsSDKVersion};$ENV{CC};$ENV{CXX};$ENV{CFLAGS};$ENV{CXXFLAGS}")
string(APPEND identity ";$ENV{RUNNER_OS};$ENV{RUNNER_ARCH};$ENV{CACHE_LANE};$ENV{CACHE_BUILD_TYPE};$ENV{CACHE_CONFIGURATION}")
execute_process(COMMAND "$ENV{CACHE_TOOL}" --version OUTPUT_VARIABLE cache_version RESULT_VARIABLE cache_status)
if(NOT cache_status EQUAL 0)
  message(FATAL_ERROR "Compiler cache tool is unavailable")
endif()
string(APPEND identity ";${cache_version}")
string(SHA256 fingerprint "${identity}")
file(WRITE "${CMAKE_BINARY_DIR}/fingerprint" "${fingerprint}")
message(STATUS "Compiler cache toolchain: ${identity}")
CMAKE
    cmake -S "$probe" -B "$probe/build" -G Ninja
    fingerprint=$(cat "$probe/build/fingerprint")
    rm -rf "$probe"
    # Immutable snapshots rotate weekly, rather than producing an archive per
    # commit or per compiler object. Restore only within the exact namespace.
    prefix="compiler-v1-${CACHE_TOOL}-${RUNNER_OS}-${RUNNER_ARCH}-${fingerprint}-"
    key="${prefix}$(date -u +%G-W%V)"
    directory="$RUNNER_TEMP/gtsam-compiler-cache"
    if [ "$RUNNER_OS" = Windows ]; then
      directory=$(cygpath -m "$directory")
    fi
    mkdir -p "$directory"
    {
      echo "COMPILER_CACHE_TOOL=$CACHE_TOOL"
      echo "COMPILER_CACHE_DIR=$directory"
      echo "COMPILER_CACHE_PREFIX=$prefix"
      echo "COMPILER_CACHE_KEY=$key"
      echo "CCACHE_DIR=$directory"
      echo "CCACHE_BASEDIR=$GITHUB_WORKSPACE"
      echo 'CCACHE_COMPILERCHECK=content'
      echo 'CCACHE_MAXSIZE=256M'
      echo "SCCACHE_DIR=$directory"
      echo 'SCCACHE_CACHE_SIZE=256M'
      echo 'SCCACHE_IDLE_TIMEOUT=0'
      # Explicit local backend; actions/cache owns all remote writes.
      echo 'SCCACHE_GHA_ENABLED=false'
    } >> "$GITHUB_ENV"
    ;;
  reset)
    echo "COMPILER_CACHE_HIT=${CACHE_HIT:-false}" >> "$GITHUB_ENV"
    if [ "$COMPILER_CACHE_TOOL" = ccache ]; then
      ccache -M 256M
      ccache -z
    else
      sccache --zero-stats
    fi
    ;;
  finish)
    if [ "$COMPILER_CACHE_TOOL" = ccache ]; then
      ccache -c
      statistics=$(ccache -s -v; echo; ccache --print-stats)
    else
      statistics=$(sccache --show-stats)
      # Flush the disk backend before taking its snapshot.
      sccache --stop-server
    fi
    size=$(du -sk "$COMPILER_CACHE_DIR" | cut -f1)
    echo "$statistics"
    echo "Compiler cache disk size: $size KiB (limit 256M)"
    {
      echo '### Compiler cache'
      echo
      echo "Key: \`$COMPILER_CACHE_KEY\`"
      echo "Exact archive hit: ${COMPILER_CACHE_HIT:-false}; disk size: $size KiB; limit: 256M."
      echo
      echo '```text'
      echo "$statistics"
      echo '```'
    } >> "$GITHUB_STEP_SUMMARY"
    ;;
  *) echo "Unknown cache operation: $1" >&2; exit 2 ;;
esac
