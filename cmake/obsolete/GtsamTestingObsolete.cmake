
# Macro for adding categorized tests in a "tests" folder, with
# optional exclusion of tests and convenience library linking options
#
# By default, all tests are linked with CppUnitLite and boost
# Arguments:
#   - subdir    The name of the category for this test
#   - local_libs  A list of convenience libraries to use (if GTSAM_BUILD_CONVENIENCE_LIBRARIES is true)
#   - full_libs   The main library to link against if not using convenience libraries
#   - excluded_tests  A list of test files that should not be compiled - use for debugging
function(gtsam_add_subdir_tests subdir local_libs full_libs excluded_tests)
    # Subdirectory target for tests
    add_custom_target(check.${subdir} COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> --output-on-failure)
    set(is_test TRUE)

	# Put check target in Visual Studio solution folder
	file(RELATIVE_PATH relative_path "${GTSAM_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
	set_property(TARGET check.${subdir} PROPERTY FOLDER "${relative_path}")

	# Link with CppUnitLite - pulled from gtsam installation
	list(APPEND local_libs CppUnitLite)
	list(APPEND full_libs CppUnitLite)

    # Build grouped tests
    gtsam_add_grouped_scripts("${subdir}"               # Use subdirectory as group label
    "tests/test*.cpp" check "Test"                      # Standard for all tests
    "${local_libs}"
    "${full_libs}" "${excluded_tests}"  # Pass in linking and exclusion lists
    ${is_test})                                         # Set all as tests
endfunction()

# Macro for adding categorized timing scripts in a "tests" folder, with
# optional exclusion of tests and convenience library linking options
#
# By default, all tests are linked with boost
# Arguments:
#   - subdir    The name of the category for this timing script
#   - local_libs  A list of convenience libraries to use (if GTSAM_BUILD_CONVENIENCE_LIBRARIES is true)
#   - full_libs   The main library to link against if not using convenience libraries
#   - excluded_srcs  A list of timing files that should not be compiled - use for debugging
macro(gtsam_add_subdir_timing subdir local_libs full_libs excluded_srcs)
    # Subdirectory target for timing - does not actually execute the scripts
    add_custom_target(timing.${subdir})
    set(is_test FALSE)

    # Build grouped benchmarks
    gtsam_add_grouped_scripts("${subdir}"               # Use subdirectory as group label
    "tests/time*.cpp" timing "Timing Benchmark"         # Standard for all timing scripts
    "${local_libs}" "${full_libs}" "${excluded_srcs}"   # Pass in linking and exclusion lists
    ${is_test})                                         # Treat as not a test
endmacro()

# Macro for adding executables matching a pattern - builds one executable for
# each file matching the pattern.  These exectuables are automatically linked
# with boost.
# Arguments:
#   - pattern    The glob pattern to match source files
#   - local_libs A list of convenience libraries to use (if GTSAM_BUILD_CONVENIENCE_LIBRARIES is true)
#   - full_libs  The main library to link against if not using convenience libraries
#   - excluded_srcs  A list of timing files that should not be compiled - use for debugging
function(gtsam_add_executables pattern local_libs full_libs excluded_srcs)
    set(is_test FALSE)

    if(NOT excluded_srcs)
        set(excluded_srcs "")
    endif()

    # Build executables
    gtsam_add_grouped_scripts("" "${pattern}" "" "Executable" "${local_libs}" "${full_libs}" "${excluded_srcs}" ${is_test})
endfunction()

# General-purpose script for adding tests with categories and linking options
macro(gtsam_add_grouped_scripts group pattern target_prefix pretty_prefix_name local_libs full_libs excluded_srcs is_test)
	# Print warning about using this obsolete function
	message(AUTHOR_WARNING "Warning:  Please see GtsamTesting.cmake - obsolete cmake cmake macro for creating unit tests, examples, and scripts was called.  This will be removed in the future.  The new macros are much easier anyway!!")

    # Get all script files
    set(script_files "")
    foreach(one_pattern ${pattern})
        file(GLOB one_script_files "${one_pattern}")
        list(APPEND script_files "${one_script_files}")
    endforeach()

    # Remove excluded scripts from the list
    set(exclusions "") # Need to copy out exclusion list for logic to work
    foreach(one_exclusion ${excluded_srcs})
        file(GLOB one_exclusion_srcs "${one_exclusion}")
        list(APPEND exclusions "${one_exclusion_srcs}")
    endforeach()
    if(exclusions)
    	list(REMOVE_ITEM script_files ${exclusions})
    endif(exclusions)

	# Separate into source files and headers
	set(script_srcs "")
	set(script_headers "")
	foreach(script_file ${script_files})
		get_filename_component(script_ext ${script_file} EXT)
		if(script_ext MATCHES "(h|H)")
			list(APPEND script_headers ${script_file})
		else()
			list(APPEND script_srcs ${script_file})
		endif()
	endforeach()


    # Add targets and dependencies for each script
    if(NOT "${group}" STREQUAL "")
        message(STATUS "Adding ${pretty_prefix_name}s in ${group}")
    endif()

	# Create exe's for each script, unless we're in SINGLE_TEST_EXE mode
	if(NOT is_test OR NOT GTSAM_SINGLE_TEST_EXE)
		foreach(script_src ${script_srcs})
			get_filename_component(script_base ${script_src} NAME_WE)
			if (script_base) # Check for null filenames and headers
				set( script_bin ${script_base} )
				message(STATUS "Adding ${pretty_prefix_name} ${script_bin}")
				add_executable(${script_bin} ${script_src} ${script_headers})
				if(NOT "${target_prefix}" STREQUAL "")
					if(NOT "${group}" STREQUAL "")
						add_dependencies(${target_prefix}.${group} ${script_bin})
					endif()
					add_dependencies(${target_prefix} ${script_bin})
				endif()

				# Add TOPSRCDIR
				set_property(SOURCE ${script_src} APPEND PROPERTY COMPILE_DEFINITIONS "TOPSRCDIR=\"${GTSAM_SOURCE_DIR}\"")

				# Disable building during make all/install
				if (GTSAM_DISABLE_TESTS_ON_INSTALL)
					set_target_properties(${script_bin} PROPERTIES EXCLUDE_FROM_ALL ON)
				endif()

				if (is_test)
					add_test(NAME ${script_base} COMMAND ${script_bin})
				endif()

				# Linking and dependendencies
				if (GTSAM_BUILD_CONVENIENCE_LIBRARIES)
					target_link_libraries(${script_bin} ${local_libs} ${GTSAM_BOOST_LIBRARIES})
				else()
					target_link_libraries(${script_bin} ${full_libs} ${GTSAM_BOOST_LIBRARIES})
				endif()

				# Add .run target
				if(NOT MSVC AND NOT XCODE_VERSION)
				  add_custom_target(${script_bin}.run ${EXECUTABLE_OUTPUT_PATH}${script_bin} ${ARGN})
				endif()

				# Set up Visual Studio folders
				file(RELATIVE_PATH relative_path "${GTSAM_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
				set_property(TARGET ${script_bin} PROPERTY FOLDER "${relative_path}")
			endif()
		endforeach(script_src)

		if(MSVC)
			source_group("" FILES ${script_srcs} ${script_headers})
		endif()
	else()
		# Create single unit test exe from all test scripts
		set(script_bin ${target_prefix}_${group}_prog)
		add_executable(${script_bin} ${script_srcs} ${script_headers})
		if (GTSAM_BUILD_CONVENIENCE_LIBRARIES)
			target_link_libraries(${script_bin} ${local_libs} ${Boost_LIBRARIES})
		else()
			target_link_libraries(${script_bin} ${Boost_LIBRARIES} ${full_libs})
		endif()

		# Only have a main function in one script
		set(rest_script_srcs ${script_srcs})
		list(REMOVE_AT rest_script_srcs 0)
		set_property(SOURCE ${rest_script_srcs} APPEND PROPERTY COMPILE_DEFINITIONS "main=static no_main")

		# Add TOPSRCDIR
		set_property(SOURCE ${script_srcs} APPEND PROPERTY COMPILE_DEFINITIONS "TOPSRCDIR=\"${GTSAM_SOURCE_DIR}\"")

		# Add test
		add_dependencies(${target_prefix}.${group} ${script_bin})
		add_dependencies(${target_prefix} ${script_bin})
		add_test(NAME ${target_prefix}.${group} COMMAND ${script_bin})

		# Disable building during make all/install
		if (GTSAM_DISABLE_TESTS_ON_INSTALL)
			set_target_properties(${script_bin} PROPERTIES EXCLUDE_FROM_ALL ON)
		endif()

		# Set up Visual Studio folders
		if(MSVC)
		    file(RELATIVE_PATH relative_path "${GTSAM_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
			set_property(TARGET ${script_bin} PROPERTY FOLDER "${relative_path}")
			source_group("" FILES ${script_srcs} ${script_headers})
		endif()
	endif()
endmacro()
