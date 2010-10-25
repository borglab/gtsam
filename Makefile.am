#----------------------------------------------------------------------------------------------------
# GTSAM top-level automake file
#----------------------------------------------------------------------------------------------------

#The option -I m4 tells Autoconf to look for additional Autoconf macros in the m4 subdirectory.
ACLOCAL_AMFLAGS = -I m4

# make automake install some standard but missing files
# also use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = foreign nostdinc

# All the sub-directories that need to be built
SUBDIRS = CppUnitLite gtsam tests examples

# Add these files to make sure they're in the distribution
EXTRA_DIST = autogen.sh configure.ac COPYING INSTALL LGPL LICENSE README THANKS USAGE
