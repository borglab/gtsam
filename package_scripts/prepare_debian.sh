#!/bin/bash
# Prepare to build a Debian package.
# Jose Luis Blanco Claraco, 2019 (for GTSAM)
# Jose Luis Blanco Claraco, 2008-2018 (for MRPT)

set -e   # end on error
#set -x  # for debugging

APPEND_SNAPSHOT_NUM=0
IS_FOR_UBUNTU=0
APPEND_LINUX_DISTRO=""
VALUE_EXTRA_CMAKE_PARAMS=""
while getopts "sud:c:" OPTION
do
     case $OPTION in
         s)
             APPEND_SNAPSHOT_NUM=1
             ;;
         u)
             IS_FOR_UBUNTU=1
             ;;
         d)
             APPEND_LINUX_DISTRO=$OPTARG
             ;;
         c)
             VALUE_EXTRA_CMAKE_PARAMS=$OPTARG
             ;;
         ?)
             echo "Unknown command line argument!"
             exit 1
             ;;
     esac
done

if [ -f CMakeLists.txt ];
then
  source package_scripts/prepare_debian_gen_snapshot_version.sh
else
	echo "Error: cannot find CMakeList.txt. This script is intended to be run from the root of the source tree."
	exit 1
fi

# Append snapshot?
if [ $APPEND_SNAPSHOT_NUM == "1" ];
then
        CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
        source $CUR_SCRIPT_DIR/prepare_debian_gen_snapshot_version.sh  # populate GTSAM_SNAPSHOT_VERSION

        GTSAM_VERSION_STR="${GTSAM_VERSION_STR}~snapshot${GTSAM_SNAPSHOT_VERSION}${APPEND_LINUX_DISTRO}"
else
        GTSAM_VERSION_STR="${GTSAM_VERSION_STR}${APPEND_LINUX_DISTRO}"
fi

# Call prepare_release
GTSAMSRC=`pwd`

if [ -f $HOME/gtsam_release/gtsam*.tar.gz ];
then
  echo "## release file already exists. Reusing it."
else
  source package_scripts/prepare_release.sh
  echo
  echo "## Done prepare_release.sh"
fi

echo "=========== Generating GTSAM ${GTSAM_VER_MMP} Debian package =============="
cd $GTSAMSRC

set -x
if [ -z "$GTSAM_DEB_DIR" ]; then
        GTSAM_DEB_DIR="$HOME/gtsam_debian"
fi
GTSAM_EXTERN_DEBIAN_DIR="$GTSAMSRC/debian/"
GTSAM_EXTERN_UBUNTU_PPA_DIR="$GTSAMSRC/debian/"

if [ -f ${GTSAM_EXTERN_DEBIAN_DIR}/control ];
then
	echo "Using debian dir: ${GTSAM_EXTERN_DEBIAN_DIR}"
else
	echo "ERROR: Cannot find ${GTSAM_EXTERN_DEBIAN_DIR}"
	exit 1
fi

GTSAM_DEBSRC_DIR=$GTSAM_DEB_DIR/gtsam-${GTSAM_VERSION_STR}

echo "GTSAM_VERSION_STR: ${GTSAM_VERSION_STR}"
echo "GTSAM_DEBSRC_DIR: ${GTSAM_DEBSRC_DIR}"

# Prepare a directory for building the debian package:
#
rm -fR $GTSAM_DEB_DIR || true
mkdir -p $GTSAM_DEB_DIR  || true

# Orig tarball:
echo "Copying orig tarball: gtsam_${GTSAM_VERSION_STR}.orig.tar.gz"
cp $HOME/gtsam_release/gtsam*.tar.gz $GTSAM_DEB_DIR/gtsam_${GTSAM_VERSION_STR}.orig.tar.gz
cd ${GTSAM_DEB_DIR}
tar -xf gtsam_${GTSAM_VERSION_STR}.orig.tar.gz

if [ ! -d "${GTSAM_DEBSRC_DIR}" ];
then
  mv gtsam-* ${GTSAM_DEBSRC_DIR}  # fix different dir names for Ubuntu PPA packages
fi

if [ ! -f "${GTSAM_DEBSRC_DIR}/CMakeLists.txt" ];
then
	echo "*ERROR*: Seems there was a problem copying sources to ${GTSAM_DEBSRC_DIR}... aborting script."
	exit 1
fi

cd ${GTSAM_DEBSRC_DIR}

# Copy debian directory:
#mkdir debian
cp -r ${GTSAM_EXTERN_DEBIAN_DIR}/* debian

# Use modified control & rules files for Ubuntu PPA packages:
#if [ $IS_FOR_UBUNTU == "1" ];
#then
	# already done: cp ${GTSAM_EXTERN_UBUNTU_PPA_DIR}/control.in debian/
  # Ubuntu: force use of gcc-7:
  #sed -i '9i\export CXX=/usr/bin/g++-7\' debian/rules
  #sed -i '9i\export CC=/usr/bin/gcc-7\' debian/rules7
#fi

# Export signing pub key:
mkdir debian/upstream/
gpg --export --export-options export-minimal --armor > debian/upstream/signing-key.asc

# Parse debian/ control.in --> control
#mv debian/control.in debian/control
#sed -i "s/@GTSAM_VER_MM@/${GTSAM_VER_MM}/g" debian/control

# Replace the text "REPLACE_HERE_EXTRA_CMAKE_PARAMS" in the "debian/rules" file
# with: ${${VALUE_EXTRA_CMAKE_PARAMS}}
RULES_FILE=debian/rules
sed -i -e "s/REPLACE_HERE_EXTRA_CMAKE_PARAMS/${VALUE_EXTRA_CMAKE_PARAMS}/g" $RULES_FILE
echo "Using these extra parameters for CMake: '${VALUE_EXTRA_CMAKE_PARAMS}'"

# Strip my custom files...
rm debian/*.new || true


# Figure out the next Debian version number:
echo "Detecting next Debian version number..."

CHANGELOG_UPSTREAM_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*.*snapshot.*\)-.*/\1/p' )
CHANGELOG_LAST_DEBIAN_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*\).*-\([0-9]*\).*/\2/p' )

echo " -> PREVIOUS UPSTREAM: $CHANGELOG_UPSTREAM_VER -> New: ${GTSAM_VERSION_STR}"
echo " -> PREVIOUS DEBIAN VERSION: $CHANGELOG_LAST_DEBIAN_VER"

# If we have the same upstream versions, increase the Debian version, otherwise create a new entry:
if [ "$CHANGELOG_UPSTREAM_VER" = "$GTSAM_VERSION_STR" ];
then
	NEW_DEBIAN_VER=$[$CHANGELOG_LAST_DEBIAN_VER + 1]
	echo "Changing to a new Debian version: ${GTSAM_VERSION_STR}-${NEW_DEBIAN_VER}"
	DEBCHANGE_CMD="--newversion ${GTSAM_VERSION_STR}-${NEW_DEBIAN_VER}"
else
	DEBCHANGE_CMD="--newversion ${GTSAM_VERSION_STR}-1"
fi

echo "Adding a new entry to debian/changelog..."

DEBEMAIL="Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD -b --distribution unstable --force-distribution New version of upstream sources.

echo "Copying back the new changelog to a temporary file in: ${GTSAM_EXTERN_DEBIAN_DIR}changelog.new"
cp debian/changelog ${GTSAM_EXTERN_DEBIAN_DIR}changelog.new

set +x

echo "=============================================================="
echo "Now, you can build the source Deb package with 'debuild -S -sa'"
echo "=============================================================="

cd ..
ls -lh

exit 0
