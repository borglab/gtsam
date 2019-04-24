###############################################################################
# Macro:
#
# gtsamAddPch(precompiledHeader precompiledSource sources)
# 
# Adds a precompiled header to compile all sources with. Currently only on MSVC.
# Inspired by https://stackoverflow.com/questions/148570/
#
# Arguments:
#   precompiledHeader: the header file that includes headers to be precompiled.
#   precompiledSource: the source file that simply includes that header above.
#   sources: the list of source files to apply this to.
#
macro(gtsamAddPch precompiledHeader precompiledSource sources)
    get_filename_component(pchBasename ${precompiledHeader} NAME_WE)
    SET(precompiledBinary "${CMAKE_CURRENT_BINARY_DIR}/${pchBasename}.pch")
	IF(MSVC)
		message(STATUS "Adding precompiled header for MSVC")
		set_source_files_properties(${precompiledSource}
									PROPERTIES COMPILE_FLAGS "/Yc\"${precompiledHeader}\" /Fp\"${precompiledBinary}\""
											   OBJECT_OUTPUTS "${precompiledBinary}")
		set_source_files_properties(${sources}
									PROPERTIES COMPILE_FLAGS "/Yu\"${precompiledHeader}\" /FI\"${precompiledHeader}\" /Fp\"${precompiledBinary}\""
											   OBJECT_DEPENDS "${precompiledBinary}")  
	ENDIF(MSVC)
endmacro(gtsamAddPch)

