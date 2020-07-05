# This list governs the order in which the JavaScript sources are
# concatenated.  This shouldn't be changed.
set (JS_MODULES Math Geodesic GeodesicLine PolygonArea DMS)

# Combine JavaScript into a single file if necessary
set (JSSCRIPTS)
set (JS_BUILD 0)
set (JS_BUILD_MIN 0)
set (JS_TARGET "${CMAKE_CURRENT_BINARY_DIR}/geographiclib.js")
set (JS_TARGET_MIN "${CMAKE_CURRENT_BINARY_DIR}/geographiclib.min.js")
set (JS_TARGETS ${JS_TARGET} ${JS_TARGET_MIN})
set (FILE_INVENTORY "")
foreach (_F ${JS_MODULES})
  set (_S "src/${_F}.js")
  set (FILE_INVENTORY "${FILE_INVENTORY} ${_F}.js")
  list (APPEND JSSCRIPTS  ${_S})
  if ("${CMAKE_CURRENT_SOURCE_DIR}/_S" IS_NEWER_THAN ${JS_TARGET})
    set (JS_BUILD 1)
  endif ()
  if ("${CMAKE_CURRENT_SOURCE_DIR}/_S" IS_NEWER_THAN ${JS_TARGET_MIN})
    set (JS_BUILD_MIN 1)
  endif ()
endforeach ()

if (JS_BUILD)
  file (STRINGS "src/Math.js" _S REGEX version_string)
  string (REGEX REPLACE ".*\"(.*)\".*" "\\1" JS_VERSION "${_S}")
  file (REMOVE ${JS_TARGET})
  file (READ "HEADER.js" _S)
  string (CONFIGURE ${_S} _S @ONLY)
  file (APPEND ${JS_TARGET} "${_S}")
  file (APPEND ${JS_TARGET} "\n(function(cb) {\n")
  foreach (_F ${JSSCRIPTS})
    get_filename_component (_N ${_F} NAME)
    file (READ "${_F}" _S)
    # Normalize the line endings.
    string (REGEX REPLACE "\r" "" _S "${_S}")
    file (APPEND ${JS_TARGET} "\n/**************** ${_N} ****************/\n")
    file (APPEND ${JS_TARGET} "${_S}")
  endforeach ()
  # export GeographlicLib
  file (APPEND ${JS_TARGET} "
cb(GeographicLib);

})(function(geo) {
  if (typeof module === 'object' && module.exports) {
    /******** support loading with node's require ********/
    module.exports = geo;
  } else if (typeof define === 'function' && define.amd) {
    /******** support loading with AMD ********/
    define('geographiclib', [], function() { return geo; });
  } else {
    /******** otherwise just pollute our global namespace ********/
    window.GeographicLib = geo;
  }
});
")
endif ()

if (JS_BUILD_MIN)
  file (STRINGS "src/Math.js" _S REGEX version_string)
  string (REGEX REPLACE ".*\"(.*)\".*" "\\1" JS_VERSION "${_S}")
  file (REMOVE ${JS_TARGET_MIN})
  file (READ "HEADER.js" _S)
  string (CONFIGURE ${_S} _S @ONLY)
  file (APPEND ${JS_TARGET_MIN} "${_S}")
  file (APPEND ${JS_TARGET_MIN} "(function(cb){\n")
  foreach (_F ${JSSCRIPTS})
    get_filename_component (_N ${_F} NAME)
    file (READ "${_F}" _S)
    # Normalize the line endings.
    string (REGEX REPLACE "\r" "\n" _S "${_S}")
    # This matches /*...*/ style comments, where ... is any number of
    # \*[^/] and [^*].  This has the defect that the it won't detect,
    # e.g., **/ as the end of the comment.
    string (REGEX REPLACE "/\\*(\\*[^/]|[^*])*\\*/" "" _S "${_S}")
    string (REGEX REPLACE "//[^\n]*\n" "\n" _S "${_S}")
    string (REGEX REPLACE "[ \t]+" " " _S "${_S}")
    string (REGEX REPLACE "([^\"A-Za-z0-9_]) " "\\1" _S "${_S}")
    string (REGEX REPLACE " ([^\\[\"A-Za-z0-9_])"  "\\1" _S "${_S}")
    string (REGEX REPLACE "\n " "\n" _S "${_S}")
    string (REGEX REPLACE " \n" "\n" _S "${_S}")
    string (REGEX REPLACE "^\n" "" _S "${_S}")
    string (REGEX REPLACE "\n+" "\n" _S "${_S}")
    file (APPEND ${JS_TARGET_MIN} "// ${_N}\n${_S}")
  endforeach ()
  # export GeographlicLib
  file (APPEND ${JS_TARGET_MIN}
    "cb(GeographicLib);
})(function(geo){
if(typeof module==='object'&&module.exports){
module.exports=geo;
}else if(typeof define==='function'&&define.amd){
define('geographiclib',[],function(){return geo;});
}else{
window.GeographicLib=geo;
}
});
")
endif ()

# "make javascript" will reconfigure cmake if necessary, since
# geographiclib.js and geographiclib.min.js are created during
# configuration.
add_custom_command (OUTPUT ${JS_TARGETS}
  DEPENDS ${JSSCRIPTS} HEADER.js
  COMMAND ${CMAKE_COMMAND} ARGS "."
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR})

file (GLOB SAMPLES samples/*.html)
file (COPY ${SAMPLES} DESTINATION .)

add_custom_target (javascript ALL DEPENDS ${JS_TARGETS})

# Copy files so that publishing nodejs package can be done with:
#   npm publish ${CMAKE_CURRENT_BINARY_DIR}/geographiclib
# To test, do:
#   cd ${CMAKE_CURRENT_BINARY_DIR}/geographiclib && npm test
file (COPY ../LICENSE.txt package.json README.md ${JS_TARGETS}
  DESTINATION geographiclib)
file (COPY ${JSSCRIPTS} DESTINATION geographiclib/src)
file (COPY test/geodesictest.js DESTINATION geographiclib/test)

if (COMMON_INSTALL_PATH)
  set (INSTALL_JS_DIR "lib${LIB_SUFFIX}/node_modules/geographiclib")
else ()
  set (INSTALL_JS_DIR "node_modules/geographiclib")
endif ()

# Install the JavaScript files
install (FILES ../LICENSE.txt package.json README.md ${JS_TARGETS}
  DESTINATION ${INSTALL_JS_DIR})
install (FILES ${JSSCRIPTS} DESTINATION ${INSTALL_JS_DIR}/src)
install (FILES test/geodesictest.js DESTINATION ${INSTALL_JS_DIR}/test)
