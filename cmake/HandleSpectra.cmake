###############################################################################
# Option for using system Spectra or GTSAM-bundled Spectra
option(GTSAM_USE_SYSTEM_SPECTRA "Find and use system-installed Spectra. If 'off', use the one bundled with GTSAM" OFF)

if(GTSAM_USE_SYSTEM_SPECTRA)
  # GTSAM uses the SortRule API introduced in Spectra 1.0.
  find_package(Spectra 1.0 CONFIG REQUIRED)
endif()
