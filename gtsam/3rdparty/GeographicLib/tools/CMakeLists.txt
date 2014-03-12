# Build the tools...

# Where to find the *.usage files for the --help option.
include_directories (${PROJECT_BINARY_DIR}/man)
# Only needed if target_compile_definitions is not supported
add_definitions (${PROJECT_DEFINITIONS})

# Loop over all the tools, specifying the source and library.
add_custom_target (tools ALL)
foreach (TOOL ${TOOLS})

  add_executable (${TOOL} ${TOOL}.cpp)
  if (MAINTAINER)
    add_dependencies (${TOOL} usage)
  endif ()
  add_dependencies (tools ${TOOL})

  set_source_files_properties (${TOOL}.cpp PROPERTIES
    OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/man/${TOOL}.usage)

  target_link_libraries (${TOOL} ${PROJECT_LIBRARIES})

endforeach ()

if (MSVC)
  get_target_property (_LIBTYPE ${PROJECT_LIBRARIES} TYPE)
  if (_LIBTYPE STREQUAL "SHARED_LIBRARY")
    # Copy the shared library on Windows systems to this directory
    # (examples) so that the tests can be run.
    add_custom_command (TARGET tools POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E
      copy $<TARGET_FILE:${PROJECT_LIBRARIES}> ${CMAKE_CFG_INTDIR}
      COMMENT "Copying shared library to tools directory")
  endif ()
endif ()

# Specify where the tools are installed
install (TARGETS ${TOOLS} EXPORT depends DESTINATION bin)

# Put all the tools into a folder in the IDE
set_property (TARGET tools ${TOOLS} PROPERTY FOLDER tools)

# Create the scripts for downloading the data files on non-Windows
# systems.  This needs to substitute ${GEOGRAPHICLIB_DATA} as the
# default data directory.  These are installed under sbin, because it is
# expected to be run with write access to /usr/local.
if (NOT WIN32)
  foreach (SCRIPT ${SCRIPTS})
    configure_file (${SCRIPT}.sh scripts/${SCRIPT} @ONLY)
    add_custom_command (OUTPUT ${SCRIPT}
      COMMAND ${CMAKE_COMMAND} -E
        copy scripts/${SCRIPT} ${SCRIPT} && chmod +x ${SCRIPT}
      DEPENDS ${SCRIPT}.sh)
    install (PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT} DESTINATION sbin)
  endforeach ()
  add_custom_target (scripts ALL DEPENDS ${SCRIPTS})
endif ()

# Turn on testing
enable_testing ()

# Here are the tests.  They consists of calling the various tools with
# --input-string and matching the output against regular expressions.

add_test (GeoConvert0 GeoConvert -p -3 -m --input-string "33.3 44.4")
set_tests_properties (GeoConvert0
  PROPERTIES PASS_REGULAR_EXPRESSION "38SMB4484")
add_test (GeoConvert1 GeoConvert -d --input-string "38smb")
set_tests_properties (GeoConvert1
  PROPERTIES PASS_REGULAR_EXPRESSION "32d59'14\\.1\"N 044d27'53\\.4\"E")
add_test (GeoConvert2 GeoConvert -p -2 --input-string "30d30'30\" 30.50833")
set_tests_properties (GeoConvert2
  PROPERTIES PASS_REGULAR_EXPRESSION "30\\.508 30\\.508")
add_test (GeoConvert3 GeoConvert --junk)
set_tests_properties (GeoConvert3 PROPERTIES WILL_FAIL ON)
add_test (GeoConvert4 GeoConvert --input-string garbage)
set_tests_properties (GeoConvert4 PROPERTIES WILL_FAIL ON)
# Check fix for DMS::Decode bug fixed on 2011-03-22
add_test (GeoConvert5 GeoConvert --input-string "5d. 0")
set_tests_properties (GeoConvert5 PROPERTIES WILL_FAIL ON)
if (NOT (MSVC AND MSVC_VERSION MATCHES "1[78].."))
  # Check fix for DMS::Decode double rounding bug fixed on 2012-11-15
  # This test is known to fail for VS 11 and 12 bug reported 2013-01-10
  # http://connect.microsoft.com/VisualStudio/feedback/details/776287
  # OK to skip this test for these compilers because this is a question
  # of accuracy of the least significant bit.
  add_test (GeoConvert6 GeoConvert -p 9
    --input-string "0 179.99999999999998578")
  set_tests_properties (GeoConvert6
    PROPERTIES PASS_REGULAR_EXPRESSION  "179\\.9999999999999[7-9]")
endif ()

add_test (GeodSolve0
  GeodSolve -i -p 0 --input-string "40.6 -73.8 49d01'N 2d33'E")
set_tests_properties (GeodSolve0
  PROPERTIES PASS_REGULAR_EXPRESSION "53\\.47022 111\\.59367 5853226")
add_test (GeodSolve1
  GeodSolve -p 0 --input-string "40d38'23\"N 073d46'44\"W 53d30' 5850e3")
set_tests_properties (GeodSolve1
  PROPERTIES PASS_REGULAR_EXPRESSION "49\\.01467 2\\.56106 111\\.62947")
# Check fix for antipodal prolate bug found 2010-09-04
add_test (GeodSolve2
  GeodSolve -i -p 0 -e 6.4e6 -1/150 --input-string "0.07476 0 -0.07476 180")
set_tests_properties (GeodSolve2
  PROPERTIES PASS_REGULAR_EXPRESSION "90\\.00078 90\\.00078 20106193")
# Another check for similar bug
add_test (GeodSolve3
  GeodSolve -i -p 0 -e 6.4e6 -1/150 --input-string "0.1 0 -0.1 180")
set_tests_properties (GeodSolve3
  PROPERTIES PASS_REGULAR_EXPRESSION "90\\.00105 90\\.00105 20106193")
# Check fix for short line bug found 2010-05-21
add_test (GeodSolve4
  GeodSolve -i --input-string "36.493349428792 0 36.49334942879201 .0000008")
set_tests_properties (GeodSolve4
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 0\\.072")
# Check fix for point2=pole bug found 2010-05-03 (but only with long double)
add_test (GeodSolve5 GeodSolve -p 0 --input-string "0.01777745589997 30 0 10e6")
set_tests_properties (GeodSolve5
  PROPERTIES PASS_REGULAR_EXPRESSION
  "90\\.00000 -150\\.00000 -180\\.00000;90\\.00000 30\\.00000 0\\.00000")

# Check fix for volatile sbet12a bug found 2011-06-25 (gcc 4.4.4 x86 -O3)
# Found again on 2012-03-27 with tdm-mingw32 (g++ 4.6.1).
add_test (GeodSolve6 GeodSolve -i --input-string
  "88.202499451857 0 -88.202499451857 179.981022032992859592")
add_test (GeodSolve7 GeodSolve -i --input-string
  "89.262080389218 0 -89.262080389218 179.992207982775375662")
add_test (GeodSolve8 GeodSolve -i --input-string
  "89.333123580033 0 -89.333123580032997687 179.99295812360148422")
set_tests_properties (GeodSolve6
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 20003898.214")
set_tests_properties (GeodSolve7
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 20003925.854")
set_tests_properties (GeodSolve8
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 20003926.881")

# Check fix for volatile x bug found 2011-06-25 (gcc 4.4.4 x86 -O3)
add_test (GeodSolve9 GeodSolve -i --input-string
  "56.320923501171 0 -56.320923501171 179.664747671772880215")
set_tests_properties (GeodSolve9
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 19993558.287")

# Check fix for adjust tol1_ bug found 2011-06-25 (Visual Studio 10 rel + debug)
add_test (GeodSolve10 GeodSolve -i --input-string
  "52.784459512564 0 -52.784459512563990912 179.634407464943777557")
set_tests_properties (GeodSolve10
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 19991596.095")

# Check fix for bet2 = -bet1 bug found 2011-06-25 (Visual Studio 10 rel + debug)
add_test (GeodSolve11 GeodSolve -i --input-string
  "48.522876735459 0 -48.52287673545898293 179.599720456223079643")
set_tests_properties (GeodSolve11
  PROPERTIES PASS_REGULAR_EXPRESSION ".* .* 19989144.774")

# Check fix for inverse geodesics on extreme prolate/oblate ellipsoids
# Reported 2012-08-29 Stefan Guenther <stefan.gunther@embl.de>; fixed 2012-10-07
add_test (GeodSolve12
  GeodSolve -i -e 89.8 -1.83 -p 0 --input-string "0 0 -10 160")
add_test (GeodSolve13
  GeodSolve -i -e 89.8 -1.83 -p 0 --input-string "0 0 -10 160" -E)
set_tests_properties (GeodSolve12 GeodSolve13
  PROPERTIES PASS_REGULAR_EXPRESSION "120\\.27.* 105\\.15.* 267")

# Check fix for pole-encircling bug found 2011-03-16
add_test (Planimeter0 Planimeter --input-string "89 0;89 90;89 180;89 270")
add_test (Planimeter1
  Planimeter -r --input-string "-89 0;-89 90;-89 180;-89 270")
add_test (Planimeter2 Planimeter --input-string "0 -1;-1 0;0 1;1 0")
add_test (Planimeter3 Planimeter --input-string "90 0; 0 0; 0 90")
add_test (Planimeter4 Planimeter -l --input-string "90 0; 0 0; 0 90")
set_tests_properties (Planimeter0
  PROPERTIES PASS_REGULAR_EXPRESSION
  "4 631819\\.8745[0-9]+ 2495230567[78]\\.[0-9]+")
set_tests_properties (Planimeter1
  PROPERTIES PASS_REGULAR_EXPRESSION
  "4 631819\\.8745[0-9]+ 2495230567[78]\\.[0-9]+")
set_tests_properties (Planimeter2
  PROPERTIES PASS_REGULAR_EXPRESSION "4 627598\\.2731[0-9]+ 24619419146.[0-9]+")
set_tests_properties (Planimeter3
  PROPERTIES PASS_REGULAR_EXPRESSION
  "3 30022685\\.[0-9]+ 63758202715511\\.[0-9]+")
set_tests_properties (Planimeter4
  PROPERTIES PASS_REGULAR_EXPRESSION "3 20020719\\.[0-9]+")
# Check fix for Planimeter pole crossing bug found 2011-06-24
add_test (Planimeter5 Planimeter --input-string "89,0.1;89,90.1;89,-179.9")
set_tests_properties (Planimeter5
  PROPERTIES PASS_REGULAR_EXPRESSION
  "3 539297\\.[0-9]+ 1247615283[89]\\.[0-9]+")
# Check fix for Planimeter lon12 rounding bug found 2012-12-03
add_test (Planimeter6 Planimeter --input-string "9 -0.00000000000001;9 180;9 0")
add_test (Planimeter7 Planimeter --input-string "9  0.00000000000001;9 0;9 180")
add_test (Planimeter8 Planimeter --input-string "9  0.00000000000001;9 180;9 0")
add_test (Planimeter9 Planimeter --input-string "9 -0.00000000000001;9 0;9 180")
set_tests_properties (Planimeter6 Planimeter7 Planimeter8 Planimeter9
  PROPERTIES PASS_REGULAR_EXPRESSION "3 36026861\\.[0-9]+ -?0.0[0-9]+")

# Check fix for AlbersEqualArea::Reverse bug found 2011-05-01
add_test (ConicProj0
  ConicProj -a 40d58 39d56 -l 77d45W -r --input-string "220e3 -52e3")
set_tests_properties (ConicProj0
  PROPERTIES PASS_REGULAR_EXPRESSION
  "39\\.95[0-9]+ -75\\.17[0-9]+ 1\\.67[0-9]+ 0\\.99[0-9]+")
# Check fix for AlbersEqualArea prolate bug found 2012-05-15
add_test (ConicProj1
  ConicProj -a 0 0 -e 6.4e6 -0.5 -r --input-string "0 8605508")
set_tests_properties (ConicProj1
  PROPERTIES PASS_REGULAR_EXPRESSION "^85\\.00")
# Check fix for LambertConformalConic::Forward bug found 2012-07-14
add_test (ConicProj2 ConicProj -c -30 -30 --input-string "-30 0")
set_tests_properties (ConicProj2
  PROPERTIES PASS_REGULAR_EXPRESSION "^-?0\\.0+ -?0\\.0+ -?0\\.0+ 1\\.0+")
# Check fixes for LambertConformalConic::Reverse overflow bugs found 2012-07-14
add_test (ConicProj3 ConicProj -r -c 0 0 --input-string "1113195 -1e10")
set_tests_properties (ConicProj3
  PROPERTIES PASS_REGULAR_EXPRESSION "^-90\\.0+ 10\\.00[0-9]+ ")
add_test (ConicProj4 ConicProj -r -c 0 0 --input-string "1113195 inf")
set_tests_properties (ConicProj4
  PROPERTIES PASS_REGULAR_EXPRESSION "^90\\.0+ 10\\.00[0-9]+ ")
add_test (ConicProj5 ConicProj -r -c 45 45 --input-string "0 -1e100")
set_tests_properties (ConicProj5
  PROPERTIES PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test (ConicProj6 ConicProj -r -c 45 45 --input-string "0 -inf")
set_tests_properties (ConicProj6
  PROPERTIES PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test (ConicProj7 ConicProj -r -c 90 90 --input-string "0 -1e150")
set_tests_properties (ConicProj7
  PROPERTIES PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test (ConicProj8 ConicProj -r -c 90 90 --input-string "0 -inf")
set_tests_properties (ConicProj8
  PROPERTIES PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")

add_test (CartConvert0
  CartConvert -e 6.4e6 1/100 -r --input-string "10e3 0 1e3")
add_test (CartConvert1
  CartConvert -e 6.4e6 -1/100 -r --input-string "1e3 0 10e3")
set_tests_properties (CartConvert0
  PROPERTIES PASS_REGULAR_EXPRESSION
  "85\\.57[0-9]+ 0\\.0[0]+ -6334614\\.[0-9]+")
set_tests_properties (CartConvert1
  PROPERTIES PASS_REGULAR_EXPRESSION
  "4\\.42[0-9]+ 0\\.0[0]+ -6398614\\.[0-9]+")

# Test fix to bad meridian convergence at pole with
# TransverseMercatorExact found 2013-06-26
add_test (TransverseMercatorProj0
  TransverseMercatorProj -k 1 --input-string "90 75")
set_tests_properties (TransverseMercatorProj0
  PROPERTIES PASS_REGULAR_EXPRESSION
  "^0\\.0+ 10001965\\.7293[0-9]+ 75\\.0+ 1\\.0+")
# Test fix to bad scale at pole with TransverseMercatorExact
# found 2013-06-30 (quarter meridian = 10001965.7293127228128889202m)
add_test (TransverseMercatorProj1
  TransverseMercatorProj -k 1 -r --input-string "0 10001965.7293127228")
set_tests_properties (TransverseMercatorProj1
  PROPERTIES PASS_REGULAR_EXPRESSION "90\\.0+ 0\\.0+ 0\\.0+ (1\\.0+|0\\.9999+)")

if (EXISTS ${GEOGRAPHICLIB_DATA}/geoids/egm96-5.pgm)
  # Check fix for single-cell cache bug found 2010-11-23
  add_test (GeoidEval0 GeoidEval -n egm96-5 --input-string "0d1 0d1;0d4 0d4")
  set_tests_properties (GeoidEval0
    PROPERTIES PASS_REGULAR_EXPRESSION "^17\\.1[56]..
17\\.1[45]..")
endif ()

if (EXISTS ${GEOGRAPHICLIB_DATA}/magnetic/wmm2010.wmm)
  # Test case from WMM2010_Report.pdf, Sec 1.5, pp 14-15:
  # t = 2012.5, lat = -80, lon = 240, h = 100e3
  add_test (MagneticField0
    MagneticField -n wmm2010 -p 10 -r --input-string "2012.5 -80 240 100e3")
  add_test (MagneticField1
    MagneticField -n wmm2010 -p 10 -r -t 2012.5 --input-string "-80 240 100e3")
  add_test (MagneticField2
    MagneticField -n wmm2010 -p 10 -r -c 2012.5 -80 100e3 --input-string "240")
  set_tests_properties (MagneticField0
    PROPERTIES PASS_REGULAR_EXPRESSION
    " 5535\\.5249148687 14765\\.3703243050 -50625\\.9305478794 .*
.* 20\\.4904268023 1\\.0272592716 83\\.5313962281 ")
  set_tests_properties (MagneticField1
    PROPERTIES PASS_REGULAR_EXPRESSION
    " 5535\\.5249148687 14765\\.3703243050 -50625\\.9305478794 .*
.* 20\\.4904268023 1\\.0272592716 83\\.5313962281 ")
  set_tests_properties (MagneticField2
    PROPERTIES PASS_REGULAR_EXPRESSION
    " 5535\\.5249148687 14765\\.3703243050 -50625\\.9305478794 .*
.* 20\\.4904268023 1\\.0272592716 83\\.5313962281 ")
endif ()

if (EXISTS ${GEOGRAPHICLIB_DATA}/gravity/egm2008.egm)
  # Verify no overflow at poles with high degree model
  add_test (Gravity0 Gravity -n egm2008 -p 6 --input-string "90 110 0")
  set_tests_properties (Gravity0
    PROPERTIES PASS_REGULAR_EXPRESSION "-0\\.000146 0\\.000078 -9\\.832294")
  # Check fix for invR bug in GravityCircle found by Mathieu Peyrega on
  # 2013-04-09
  add_test (Gravity1 Gravity -n egm2008 -A -c -18 4000 --input-string "-86")
  set_tests_properties (Gravity1
    PROPERTIES PASS_REGULAR_EXPRESSION "-7\\.438 1\\.305 -1\\.563")
  add_test (Gravity2 Gravity -n egm2008 -D -c -18 4000 --input-string "-86")
  set_tests_properties (Gravity2
    PROPERTIES PASS_REGULAR_EXPRESSION "7\\.404 -6\\.168 7\\.616")
endif ()
