#===============================================================================
# SuiteSparse_config.mk:  common configuration file for the SuiteSparse
#===============================================================================

# This file contains all configuration settings for all packages in SuiteSparse,
# except for CSparse (which is stand-alone), the packages in MATLAB_Tools,
# and GraphBLAS.  The configuration settings for GraphBLAS are determined by
# GraphBLAS/CMakeLists.txt

SUITESPARSE_VERSION = 5.4.0

#===============================================================================
# Options you can change without editing this file:
#===============================================================================

    # To list the options you can modify at the 'make' command line, type
    # 'make config', which also lists their default values.  You can then
    # change them with 'make OPTION=value'.  For example, to use an INSTALL
    # path of /my/path, and to use your own BLAS and LAPACK libraries, do:
    #
    #   make install INSTALL=/my/path BLAS=-lmyblas LAPACK=-lmylapackgoeshere
    #
    # which will install the package into /my/path/lib and /my/path/include,
    # and use -lmyblas -lmylapackgoes here when building the demo program.

#===============================================================================
# Defaults for any system
#===============================================================================

    #---------------------------------------------------------------------------
    # SuiteSparse root directory
    #---------------------------------------------------------------------------

    # Most Makefiles are in SuiteSparse/Pkg/Lib or SuiteSparse/Pkg/Demo, so
    # the top-level of SuiteSparse is in ../.. unless otherwise specified.
    # This is true for all but the SuiteSparse_config package.
    SUITESPARSE ?= $(realpath $(CURDIR)/../..)

    #---------------------------------------------------------------------------
    # installation location
    #---------------------------------------------------------------------------

    # For "make install" and "make uninstall", the default location is
    # SuiteSparse/lib, SuiteSparse/include, and
    # SuiteSparse/share/doc/suitesparse-x.y.z
    # If you do this:
    #    make install INSTALL=/usr/local
    # then the libraries are installed in /usr/local/lib, include files in
    # /usr/local/include, and documentation in
    # /usr/local/share/doc/suitesparse-x.y.z.
    # You can instead specify the install location of each of these 3 components
    # separately, via (for example):
    #    make install INSTALL_LIB=/yada/mylibs INSTALL_INCLUDE=/yoda/myinc  \
    #                 INSTALL_DOC=/solo/mydox
    # which puts the libraries in /yada/mylibs, include files in /yoda/myinc,
    # and documentation in /solo/mydox.
    INSTALL ?= $(SUITESPARSE)
    INSTALL_LIB ?= $(INSTALL)/lib
    INSTALL_INCLUDE ?= $(INSTALL)/include
    INSTALL_DOC ?= $(INSTALL)/share/doc/suitesparse-$(SUITESPARSE_VERSION)

    CMAKE_OPTIONS ?= -DCMAKE_INSTALL_PREFIX=$(INSTALL)

    #---------------------------------------------------------------------------
    # parallel make
    #---------------------------------------------------------------------------

    # sequential make's by default
    JOBS ?= 1

    #---------------------------------------------------------------------------
    # optimization level
    #---------------------------------------------------------------------------

    OPTIMIZATION ?= -O3

    #---------------------------------------------------------------------------
    # statement coverage for */Tcov
    #---------------------------------------------------------------------------

    ifeq ($(TCOV),yes)
        # Each package has a */Tcov directory for extensive testing, including
        # statement coverage.  The Tcov tests require Linux and gcc, and use
        # the vanilla BLAS.  For those tests, the packages use 'make TCOV=yes',
        # which overrides the following settings:
        MKLROOT =
        AUTOCC = no
        CC = gcc
        CXX = g++
        BLAS = -lrefblas -lgfortran -lstdc++
        LAPACK = -llapack
        CFLAGS += --coverage
        OPTIMIZATION = -g
        LDFLAGS += --coverage
    endif

    #---------------------------------------------------------------------------
    # OpenMP is used in CHOLMOD
    #---------------------------------------------------------------------------

    # with gcc, enable OpenMP directives via -fopenmp
    # This is not supported on Darwin, so this string is cleared, below.
    CFOPENMP ?= -fopenmp

    #---------------------------------------------------------------------------
    # compiler
    #---------------------------------------------------------------------------

    # By default, look for the Intel compilers.  If present, they are used
    # instead of $(CC), $(CXX), and $(F77).  To disable this feature and
    # use the $(CC), $(CXX), and $(F77) compilers, use 'make AUTOCC=no'

    AUTOCC ?= yes

    ifneq ($(AUTOCC),no)
        ifneq ($(shell which icc 2>/dev/null),)
            # use the Intel icc compiler for C codes, and -qopenmp for OpenMP
            CC = icc
            CFLAGS += -D_GNU_SOURCE
            CXX = icpc
            CFOPENMP = -qopenmp -I$(MKLROOT)/include
            LDFLAGS += -qopenmp
            LDLIBS += -lm -lirc
        endif
        ifneq ($(shell which ifort 2>/dev/null),)
            # use the Intel ifort compiler for Fortran codes
            F77 = ifort
        endif
    endif

    CMAKE_OPTIONS += -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC)

    #---------------------------------------------------------------------------
    # CFLAGS for the C/C++ compiler
    #---------------------------------------------------------------------------

    # The CF macro is used by SuiteSparse Makefiles as a combination of
    # CFLAGS, CPPFLAGS, TARGET_ARCH, and system-dependent settings.
    CF ?= $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) $(OPTIMIZATION) -fexceptions -fPIC

    #---------------------------------------------------------------------------
    # code formatting (for Tcov on Linux only)
    #---------------------------------------------------------------------------

    PRETTY ?= grep -v "^\#" | indent -bl -nce -bli0 -i4 -sob -l120

    #---------------------------------------------------------------------------
    # required libraries
    #---------------------------------------------------------------------------

    # SuiteSparse requires the BLAS, LAPACK, and -lm (Math) libraries.
    # It places its shared *.so libraries in SuiteSparse/lib.
    # Linux also requires the -lrt library (see below)
    LDLIBS ?= -lm
    LDFLAGS += -L$(INSTALL_LIB)

    # See http://www.openblas.net for a recent and freely available optimzed
    # BLAS.  LAPACK is at http://www.netlib.org/lapack/ .  You can use the
    # standard Fortran LAPACK along with OpenBLAS to obtain very good
    # performance.  This script can also detect if the Intel MKL BLAS is
    # installed.

    LAPACK ?= -llapack

    ifndef BLAS
        ifdef MKLROOT
            # use the Intel MKL for BLAS and LAPACK
            # using static linking:
            # BLAS = -Wl,--start-group \
            #   $(MKLROOT)/lib/intel64/libmkl_intel_lp64.a \
            #   $(MKLROOT)/lib/intel64/libmkl_core.a \
            #   $(MKLROOT)/lib/intel64/libmkl_intel_thread.a \
            #   -Wl,--end-group -lpthread -lm
            # using dynamic linking:
            BLAS = -lmkl_intel_lp64 -lmkl_core -lmkl_intel_thread -liomp5 -lpthread -lm
            LAPACK =
        else
            # use the OpenBLAS at http://www.openblas.net
            BLAS = -lopenblas
        endif
    endif

    # For ACML, use this instead:
    #   make BLAS='-lacml -lgfortran'

    #---------------------------------------------------------------------------
    # shell commands
    #---------------------------------------------------------------------------

    # ranlib, and ar, for generating libraries.  If you don't need ranlib,
    # just change it to RANLAB = echo
    RANLIB ?= ranlib
    ARCHIVE ?= $(AR) $(ARFLAGS)
    CP ?= cp -f
    MV ?= mv -f

    #---------------------------------------------------------------------------
    # Fortran compiler (not required for 'make' or 'make library')
    #---------------------------------------------------------------------------

    # A Fortran compiler is optional.  Only required for the optional Fortran
    # interfaces to AMD and UMFPACK.  Not needed by 'make' or 'make install'
    F77 ?= gfortran
    F77FLAGS ?= $(FFLAGS) $(OPTIMIZATION)

    #---------------------------------------------------------------------------
    # NVIDIA CUDA configuration for CHOLMOD and SPQR
    #---------------------------------------------------------------------------

    # CUDA is detected automatically, and used if found.  To disable CUDA,
    # use CUDA=no

    ifneq ($(CUDA),no)
        CUDA_PATH = $(shell which nvcc 2>/dev/null | sed "s/\/bin\/nvcc//")
    endif

    ifeq ($(wildcard $(CUDA_PATH)),)
        # CUDA is not present
        CUDA_PATH     =
        GPU_BLAS_PATH =
        GPU_CONFIG    =
        CUDART_LIB    =
        CUBLAS_LIB    =
        CUDA_INC_PATH =
        CUDA_INC      =
        NVCC          = echo
        NVCCFLAGS     =
    else
        # with CUDA for CHOLMOD and SPQR
        GPU_BLAS_PATH = $(CUDA_PATH)
        # GPU_CONFIG must include -DGPU_BLAS to compile SuiteSparse for the
        # GPU.  You can add additional GPU-related flags to it as well.
        # with 4 cores (default):
        GPU_CONFIG    = -DGPU_BLAS
        # For example, to compile CHOLMOD for 10 CPU cores when using the GPU:
        # GPU_CONFIG  = -DGPU_BLAS -DCHOLMOD_OMP_NUM_THREADS=10
        CUDART_LIB    = $(CUDA_PATH)/lib64/libcudart.so
        CUBLAS_LIB    = $(CUDA_PATH)/lib64/libcublas.so
        CUDA_INC_PATH = $(CUDA_PATH)/include/
        CUDA_INC      = -I$(CUDA_INC_PATH)
                MAGMA_INC     = -I/opt/magma-2.4.0/include/
                MAGMA_LIB     = -L/opt/magma-2.4.0/lib/ -lmagma
        NVCC          = $(CUDA_PATH)/bin/nvcc
        NVCCFLAGS     = -Xcompiler -fPIC -O3 \
                            -gencode=arch=compute_30,code=sm_30 \
                            -gencode=arch=compute_35,code=sm_35 \
                            -gencode=arch=compute_50,code=sm_50 \
                            -gencode=arch=compute_53,code=sm_53 \
                            -gencode=arch=compute_53,code=sm_53 \
                            -gencode=arch=compute_60,code=compute_60
    endif

    #---------------------------------------------------------------------------
    # UMFPACK configuration:
    #---------------------------------------------------------------------------

    # Configuration for UMFPACK.  See UMFPACK/Source/umf_config.h for details.
    #
    # -DNBLAS       do not use the BLAS.  UMFPACK will be very slow.
    # -D'LONGBLAS=long' or -DLONGBLAS='long long' defines the integers used by
    #               LAPACK and the BLAS (defaults to 'int')
    # -DNSUNPERF    do not use the Sun Perf. Library on Solaris
    # -DNRECIPROCAL do not multiply by the reciprocal
    # -DNO_DIVIDE_BY_ZERO   do not divide by zero
    # -DNCHOLMOD    do not use CHOLMOD as a ordering method.  If -DNCHOLMOD is
    #               included in UMFPACK_CONFIG, then UMFPACK does not rely on
    #               CHOLMOD, CAMD, CCOLAMD, COLAMD, and METIS.

    UMFPACK_CONFIG ?=

    # For example, uncomment this line to compile UMFPACK without CHOLMOD:
    # UMFPACK_CONFIG = -DNCHOLMOD
    # or use 'make UMFPACK_CONFIG=-DNCHOLMOD'

    #---------------------------------------------------------------------------
    # CHOLMOD configuration
    #---------------------------------------------------------------------------

    # CHOLMOD Library Modules, which appear in -lcholmod
    # Core       requires: none
    # Check      requires: Core
    # Cholesky   requires: Core, AMD, COLAMD. optional: Partition, Supernodal
    # MatrixOps  requires: Core
    # Modify     requires: Core
    # Partition  requires: Core, CCOLAMD, METIS.  optional: Cholesky
    # Supernodal requires: Core, BLAS, LAPACK
    #
    # CHOLMOD test/demo Modules (these do not appear in -lcholmod):
    # Tcov       requires: Core, Check, Cholesky, MatrixOps, Modify, Supernodal
    #            optional: Partition
    # Valgrind   same as Tcov
    # Demo       requires: Core, Check, Cholesky, MatrixOps, Supernodal
    #            optional: Partition
    #
    # Configuration flags:
    # -DNCHECK      do not include the Check module.
    # -DNCHOLESKY   do not include the Cholesky module.
    # -DNPARTITION  do not include the Partition module.
    #               also do not include METIS.
    # -DNCAMD       do not use CAMD & CCOLAMD in Parition Module.
    # -DNMATRIXOPS  do not include the MatrixOps module.
    # -DNMODIFY     do not include the Modify module.
    # -DNSUPERNODAL do not include the Supernodal module.
    #
    # -DNPRINT      do not print anything.
    # -D'LONGBLAS=long' or -DLONGBLAS='long long' defines the integers used by
    #               LAPACK and the BLAS (defaults to 'int')
    # -DNSUNPERF    for Solaris only.  If defined, do not use the Sun
    #               Performance Library
    # -DGPU_BLAS    enable the use of the CUDA BLAS

    CHOLMOD_CONFIG ?= $(GPU_CONFIG)

    #---------------------------------------------------------------------------
    # SuiteSparseQR configuration:
    #---------------------------------------------------------------------------

    # The SuiteSparseQR library can be compiled with the following options:
    #
    # -DNPARTITION      do not include the CHOLMOD partition module
    # -DNEXPERT         do not include the functions in SuiteSparseQR_expert.cpp
    # -DHAVE_TBB        enable the use of Intel's Threading Building Blocks
    # -DGPU_BLAS        enable the use of the CUDA BLAS

    SPQR_CONFIG ?= $(GPU_CONFIG)

    # to compile with Intel's TBB, use TBB=-ltbb -DSPQR_CONFIG=-DHAVE_TBB
    TBB ?=
    # TBB = -ltbb -DSPQR_CONFIG=-DHAVE_TBB

    # TODO: this *mk file should auto-detect the presence of Intel's TBB,
    # and set the compiler flags accordingly.

#===============================================================================
# System-dependent configurations
#===============================================================================

    #---------------------------------------------------------------------------
    # determine what system we are on
    #---------------------------------------------------------------------------

    # To disable these auto configurations, use 'make UNAME=custom'

    ifndef UNAME
        ifeq ($(OS),Windows_NT)
            # Cygwin Make on Windows has an $(OS) variable, but not uname.
            # Note that this option is untested.
            UNAME = Windows
        else
            # Linux and Darwin (Mac OSX) have been tested.
            UNAME := $(shell uname)
        endif
    endif

    #---------------------------------------------------------------------------
    # Linux
    #---------------------------------------------------------------------------

    ifeq ($(UNAME),Linux)
        # add the realtime library, librt, and SuiteSparse/lib
        LDLIBS += -lrt -Wl,-rpath=$(INSTALL_LIB)
    endif

    #---------------------------------------------------------------------------
    # Mac
    #---------------------------------------------------------------------------

    ifeq ($(UNAME), Darwin)
        # To compile on the Mac, you must install Xcode.  Then do this at the
        # command line in the Terminal, before doing 'make':
        # xcode-select --install
        CF += -fno-common
        BLAS = -framework Accelerate
        LAPACK = -framework Accelerate
        # OpenMP is not yet supported by default in clang
        CFOPENMP =
    endif

    #---------------------------------------------------------------------------
    # Solaris
    #---------------------------------------------------------------------------

    ifeq ($(UNAME), SunOS)
        # Using the Sun compiler and the Sun Performance Library
        # This hasn't been tested recently.
        # I leave it here in case you need it.  It likely needs updating.
        CF += -fast -KPIC -xc99=%none -xlibmieee -xlibmil -m64 -Xc
        F77FLAGS = -O -fast -KPIC -dalign -xlibmil -m64
        BLAS = -xlic_lib=sunperf
        LAPACK =
        # Using the GCC compiler and the reference BLAS
        ## CC = gcc
        ## CXX = g++
        ## MAKE = gmake
        ## BLAS = -lrefblas -lgfortran
        ## LAPACK = -llapack
    endif

    #---------------------------------------------------------------------------
    # IBM AIX
    #---------------------------------------------------------------------------

    ifeq ($(UNAME), AIX)
        # hasn't been tested for a very long time...
        # I leave it here in case you need it.  It likely needs updating.
        CF += -O4 -qipa -qmaxmem=16384 -q64 -qproto -DBLAS_NO_UNDERSCORE
        F77FLAGS =  -O4 -qipa -qmaxmem=16384 -q64
        BLAS = -lessl
        LAPACK =
    endif

#===============================================================================
# finalize the CF compiler flags
#===============================================================================

    CF += $(CFOPENMP)

#===============================================================================
# internal configuration
#===============================================================================

    # The user should not have to change these definitions, and they are
    # not displayed by 'make config'

    #---------------------------------------------------------------------------
    # for removing files not in the distribution
    #---------------------------------------------------------------------------

    # remove object files, but keep compiled libraries via 'make clean'
    CLEAN = *.o *.obj *.ln *.bb *.bbg *.da *.tcov *.gcov gmon.out *.bak *.d \
        *.gcda *.gcno *.aux *.bbl *.blg *.log *.toc *.dvi *.lof *.lot

    # also remove compiled libraries, via 'make distclean'
    PURGE = *.so* *.a *.dll *.dylib *.dSYM

    # location of TCOV test output
    TCOV_TMP ?= /tmp

#===============================================================================
# Building the shared and static libraries
#===============================================================================

# How to build/install shared and static libraries for Mac and Linux/Unix.
# This assumes that LIBRARY and VERSION have already been defined by the
# Makefile that includes this file.

SO_OPTS = $(LDFLAGS)

ifeq ($(UNAME),Windows)
    # Cygwin Make on Windows (untested)
    AR_TARGET = $(LIBRARY).lib
    SO_PLAIN  = $(LIBRARY).dll
    SO_MAIN   = $(LIBRARY).$(SO_VERSION).dll
    SO_TARGET = $(LIBRARY).$(VERSION).dll
    SO_INSTALL_NAME = echo
else
    # Mac or Linux/Unix
    AR_TARGET = $(LIBRARY).a
    ifeq ($(UNAME),Darwin)
        # Mac
        SO_PLAIN  = $(LIBRARY).dylib
        SO_MAIN   = $(LIBRARY).$(SO_VERSION).dylib
        SO_TARGET = $(LIBRARY).$(VERSION).dylib
        SO_OPTS  += -dynamiclib -compatibility_version $(SO_VERSION) \
                    -current_version $(VERSION) \
                    -shared -undefined dynamic_lookup
        # When a Mac *.dylib file is moved, this command is required
        # to change its internal name to match its location in the filesystem:
        SO_INSTALL_NAME = install_name_tool -id
    else
        # Linux and other variants of Unix
        SO_PLAIN  = $(LIBRARY).so
        SO_MAIN   = $(LIBRARY).so.$(SO_VERSION)
        SO_TARGET = $(LIBRARY).so.$(VERSION)
        SO_OPTS  += -shared -Wl,-soname -Wl,$(SO_MAIN) -Wl,--no-undefined
        # Linux/Unix *.so files can be moved without modification:
        SO_INSTALL_NAME = echo
    endif
endif

#===============================================================================
# Configure CHOLMOD/Partition module with METIS, CAMD, and CCOLAMD
#===============================================================================

# By default, SuiteSparse uses METIS 5.1.0 in the SuiteSparse/metis-5.1.0
# directory.  SuiteSparse's interface to METIS is only through the
# SuiteSparse/CHOLMOD/Partition module, which also requires SuiteSparse/CAMD
# and SuiteSparse/CCOLAMD.
#
# If you wish to use your own pre-installed copy of METIS, use the MY_METIS_LIB
# and MY_METIS_INC options passed to 'make'.  For example:
#       make MY_METIS_LIB=-lmetis
#       make MY_METIS_LIB=/home/myself/mylibraries/libmetis.so
#       make MY_METIS_LIB='-L/home/myself/mylibraries -lmetis'
# If you need to tell the compiler where to find the metis.h include file,
# then add MY_METIS_INC=/home/myself/metis-5.1.0/include as well, which points
# to the directory containing metis.h.  If metis.h is already installed in
# a location known to the compiler (/usr/local/include/metis.h for example)
# then you do not need to add MY_METIS_INC.

I_WITH_PARTITION =
LIB_WITH_PARTITION =
CONFIG_PARTITION = -DNPARTITION -DNCAMD
# check if CAMD/CCOLAMD and METIS are requested and available
ifeq (,$(findstring -DNCAMD, $(CHOLMOD_CONFIG)))
    # CAMD and CCOLAMD are requested.  See if they are available in
    # SuiteSparse/CAMD and SuiteSparse/CCOLAMD
    ifneq (, $(wildcard $(SUITESPARSE)/CAMD))
        ifneq (, $(wildcard $(SUITESPARSE)/CCOLAMD))
            # CAMD and CCOLAMD are requested and available
            LIB_WITH_PARTITION = -lccolamd -lcamd
            I_WITH_PARTITION = -I$(SUITESPARSE)/CCOLAMD/Include -I$(SUITESPARSE)/CAMD/Include
            CONFIG_PARTITION = -DNPARTITION
            # check if METIS is requested and available
            ifeq (,$(findstring -DNPARTITION, $(CHOLMOD_CONFIG)))
                # METIS is requested.  See if it is available.
                ifneq (,$(MY_METIS_LIB))
                    # METIS 5.1.0 is provided elsewhere, and we are not using
                    # SuiteSparse/metis-5.1.0. To do so, we link with
                    # $(MY_METIS_LIB) and add the -I$(MY_METIS_INC) option for
                    # the compiler.  The latter can be empty if you have METIS
                    # installed in a place where the compiler can find the
                    # metis.h include file by itself without any -I option
                    # (/usr/local/include/metis.h for example). 
                    LIB_WITH_PARTITION += $(MY_METIS_LIB)
                    ifneq (,$(MY_METIS_INC))
                        I_WITH_PARTITION += -I$(MY_METIS_INC)
                    endif
                    CONFIG_PARTITION =
                else
                    # see if METIS is in SuiteSparse/metis-5.1.0
                    ifneq (, $(wildcard $(SUITESPARSE)/metis-5.1.0))
                        # SuiteSparse/metis5.1.0 is available
                        ifeq ($(UNAME), Darwin)
                            LIB_WITH_PARTITION += $(SUITESPARSE)/lib/libmetis.dylib
                        else
                            LIB_WITH_PARTITION += -lmetis
                        endif
                        I_WITH_PARTITION += -I$(SUITESPARSE)/metis-5.1.0/include
                        CONFIG_PARTITION =
                    endif
                endif
            endif
        endif
    endif
endif

#===============================================================================
# display configuration
#===============================================================================

ifeq ($(LIBRARY),)
    # placeholders, for 'make config' in the top-level SuiteSparse
    LIBRARY=PackageNameWillGoHere
    VERSION=x.y.z
    SO_VERSION=x
endif

# 'make config' lists the primary installation options
config:
	@echo ' '
	@echo '----------------------------------------------------------------'
	@echo 'SuiteSparse package compilation options:'
	@echo '----------------------------------------------------------------'
	@echo ' '
	@echo 'SuiteSparse Version:     ' '$(SUITESPARSE_VERSION)'
	@echo 'SuiteSparse top folder:  ' '$(SUITESPARSE)'
	@echo 'Package:                  LIBRARY=        ' '$(LIBRARY)'
	@echo 'Version:                  VERSION=        ' '$(VERSION)'
	@echo 'SO version:               SO_VERSION=     ' '$(SO_VERSION)'
	@echo 'System:                   UNAME=          ' '$(UNAME)'
	@echo 'Install directory:        INSTALL=        ' '$(INSTALL)'
	@echo 'Install libraries in:     INSTALL_LIB=    ' '$(INSTALL_LIB)'
	@echo 'Install include files in: INSTALL_INCLUDE=' '$(INSTALL_INCLUDE)'
	@echo 'Install documentation in: INSTALL_DOC=    ' '$(INSTALL_DOC)'
	@echo 'Optimization level:       OPTIMIZATION=   ' '$(OPTIMIZATION)'
	@echo 'parallel make jobs:       JOBS=           ' '$(JOBS)'
	@echo 'BLAS library:             BLAS=           ' '$(BLAS)'
	@echo 'LAPACK library:           LAPACK=         ' '$(LAPACK)'
	@echo 'Intel TBB library:        TBB=            ' '$(TBB)'
	@echo 'Other libraries:          LDLIBS=         ' '$(LDLIBS)'
	@echo 'static library:           AR_TARGET=      ' '$(AR_TARGET)'
	@echo 'shared library (full):    SO_TARGET=      ' '$(SO_TARGET)'
	@echo 'shared library (main):    SO_MAIN=        ' '$(SO_MAIN)'
	@echo 'shared library (short):   SO_PLAIN=       ' '$(SO_PLAIN)'
	@echo 'shared library options:   SO_OPTS=        ' '$(SO_OPTS)'
	@echo 'shared library name tool: SO_INSTALL_NAME=' '$(SO_INSTALL_NAME)'
	@echo 'ranlib, for static libs:  RANLIB=         ' '$(RANLIB)'
	@echo 'static library command:   ARCHIVE=        ' '$(ARCHIVE)'
	@echo 'copy file:                CP=             ' '$(CP)'
	@echo 'move file:                MV=             ' '$(MV)'
	@echo 'remove file:              RM=             ' '$(RM)'
	@echo 'pretty (for Tcov tests):  PRETTY=         ' '$(PRETTY)'
	@echo 'C compiler:               CC=             ' '$(CC)'
	@echo 'C++ compiler:             CXX=            ' '$(CXX)'
	@echo 'CUDA compiler:            NVCC=           ' '$(NVCC)'
	@echo 'CUDA root directory:      CUDA_PATH=      ' '$(CUDA_PATH)'
	@echo 'OpenMP flags:             CFOPENMP=       ' '$(CFOPENMP)'
	@echo 'C/C++ compiler flags:     CF=             ' '$(CF)'
	@echo 'LD flags:                 LDFLAGS=        ' '$(LDFLAGS)'
	@echo 'Fortran compiler:         F77=            ' '$(F77)'
	@echo 'Fortran flags:            F77FLAGS=       ' '$(F77FLAGS)'
	@echo 'Intel MKL root:           MKLROOT=        ' '$(MKLROOT)'
	@echo 'Auto detect Intel icc:    AUTOCC=         ' '$(AUTOCC)'
	@echo 'UMFPACK config:           UMFPACK_CONFIG= ' '$(UMFPACK_CONFIG)'
	@echo 'CHOLMOD config:           CHOLMOD_CONFIG= ' '$(CHOLMOD_CONFIG)'
	@echo 'SuiteSparseQR config:     SPQR_CONFIG=    ' '$(SPQR_CONFIG)'
	@echo 'CUDA library:             CUDART_LIB=     ' '$(CUDART_LIB)'
	@echo 'CUBLAS library:           CUBLAS_LIB=     ' '$(CUBLAS_LIB)'
	@echo 'METIS and CHOLMOD/Partition configuration:'
	@echo 'Your METIS library:       MY_METIS_LIB=   ' '$(MY_METIS_LIB)'
	@echo 'Your metis.h is in:       MY_METIS_INC=   ' '$(MY_METIS_INC)'
	@echo 'METIS is used via the CHOLMOD/Partition module, configured as follows.'
	@echo 'If the next line has -DNPARTITION then METIS will not be used:'
	@echo 'CHOLMOD Partition config: ' '$(CONFIG_PARTITION)'
	@echo 'CHOLMOD Partition libs:   ' '$(LIB_WITH_PARTITION)'
	@echo 'CHOLMOD Partition include:' '$(I_WITH_PARTITION)'
ifeq ($(TCOV),yes)
	@echo 'TCOV=yes, for extensive testing only (gcc, g++, vanilla BLAS)'
endif

