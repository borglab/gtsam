# if GTSAM_ENABLE_BOOST_SERIALIZATION is not set then SolverComparer.cpp will not compile
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
    list (APPEND excluded_examples
    "SolverComparer.cpp"
  )
endif()

# Add examples to exclude if GTSAM_USE_BOOST_FEATURES is not set
if (NOT GTSAM_USE_BOOST_FEATURES) 
  # add to excluded examples
  list (APPEND excluded_examples
      "CombinedImuFactorsExample.cpp"
      "ImuFactorsExample.cpp"
      "ShonanAveragingCLI.cpp"
      "SolverComparer.cpp"
      )
endif()

if(NOT GTSAM_USE_MOSEK)
  list(APPEND excluded_examples
      "Pose2RingSLAM_LiftedSDP.cpp"
      "Pose3RingSLAM_LiftedSDP.cpp"
      "Rot2RingQcqpToMonolithicSDP.cpp")
endif()

find_package(Torch QUIET CONFIG)
if(NOT Torch_FOUND)
  list(APPEND excluded_examples "GradientsTorchExample.cpp")
endif()

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}" ${GTSAM_BUILD_EXAMPLES_ALWAYS})

if(Torch_FOUND)
  target_link_libraries(GradientsTorchExample ${TORCH_LIBRARIES})
  # Honor LibTorch's required standard on this example only.
  get_target_property(torch_cxx_standard torch CXX_STANDARD)
  if(torch_cxx_standard)
    target_compile_features(GradientsTorchExample PRIVATE cxx_std_${torch_cxx_standard})
  endif()
endif()
