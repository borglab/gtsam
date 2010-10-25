
# All the sub-directories that need to be built
SUBDIRS = base geometry inference linear nonlinear slam

# And the corresponding libraries produced
SUBLIBS = base/libbase.la geometry/libgeometry.la inference/libinference.la \
          linear/liblinear.la nonlinear/libnonlinear.la slam/libslam.la

# The following lines specify the actual shared library to be built with libtool
lib_LTLIBRARIES = libgtsam.la
libgtsam_la_SOURCES = 
nodist_EXTRA_libgtsam_la_SOURCES = dummy.cxx
libgtsam_la_LIBADD = $(SUBLIBS) -L$(CCOLAMDLib) $(BOOST_LDFLAGS) -lccolamd
libgtsam_la_LDFLAGS = -no-undefined -version-info 0:0:0

if USE_ACCELERATE_MACOS
libgtsam_la_LDFLAGS += -Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate
endif
