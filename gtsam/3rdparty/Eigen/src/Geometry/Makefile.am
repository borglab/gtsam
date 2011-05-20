# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = arch

# And the corresponding libraries produced
# TODO add sublibs when something in 3rdparty actually needs compilation
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers  = 

# Geometry:
headers += AlignedBox.h
headers += AngleAxis.h
headers += EulerAngles.h
headers += Homogeneous.h
headers += Hyperplane.h
headers += OrthoMethods.h
headers += ParametrizedLine.h
headers += Quaternion.h
headers += Rotation2D.h
headers += RotationBase.h
headers += Scaling.h
headers += Transform.h
headers += Translation.h
headers += Umeyama.h

Geometrydir = $(pkgincludedir)/3rdparty/Eigen/src/Geometry
Geometry_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/Geometry
Geometry_HEADERS = $(headers) 