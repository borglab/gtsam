#!/bin/bash
# Creates a set of packages for each different Ubuntu distribution, with the
# intention of uploading them to:
#   https://launchpad.net/~joseluisblancoc/
#
# JLBC, 2010
# [Addition 2012:]
#
# You can declare a variable (in the caller shell) with extra flags for the
# CMake in the final ./configure like:
#  GTSAM_PKG_CUSTOM_CMAKE_PARAMS="\"-DDISABLE_SSE3=ON\""
#

set -e

# List of distributions to create PPA packages for:
LST_DISTROS=(xenial bionic disco eoan)

# Checks
# --------------------------------
if [ -f CMakeLists.txt ];
then
	source package_scripts/prepare_debian_gen_snapshot_version.sh
	echo "GTSAM version: ${GTSAM_VER_MMP}"
else
	echo "ERROR: Run this script from the GTSAM root directory."
	exit 1
fi

if [ -z "${gtsam_ubuntu_OUT_DIR}" ]; then
       export gtsam_ubuntu_OUT_DIR="$HOME/gtsam_ubuntu"
fi
GTSAMSRC=`pwd`
if [ -z "${GTSAM_DEB_DIR}" ]; then
       export GTSAM_DEB_DIR="$HOME/gtsam_debian"
fi
GTSAM_EXTERN_DEBIAN_DIR="$GTSAMSRC/debian/"
EMAIL4DEB="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>"

# Clean out dirs:
rm -fr $gtsam_ubuntu_OUT_DIR/

# -------------------------------------------------------------------
# And now create the custom packages for each Ubuntu distribution:
# -------------------------------------------------------------------
count=${#LST_DISTROS[@]}
IDXS=$(seq 0 $(expr $count - 1))

cp ${GTSAM_EXTERN_DEBIAN_DIR}/changelog /tmp/my_changelog

for IDX in ${IDXS};
do
	DEBIAN_DIST=${LST_DISTROS[$IDX]}

	# -------------------------------------------------------------------
	# Call the standard "prepare_debian.sh" script:
	# -------------------------------------------------------------------
	cd ${GTSAMSRC}
	bash package_scripts/prepare_debian.sh -s -u -d ${DEBIAN_DIST}   -c "${GTSAM_PKG_CUSTOM_CMAKE_PARAMS}"

	CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	source $CUR_SCRIPT_DIR/prepare_debian_gen_snapshot_version.sh # populate GTSAM_SNAPSHOT_VERSION

	echo "===== Distribution: ${DEBIAN_DIST}  ========="
	cd ${GTSAM_DEB_DIR}/gtsam-${GTSAM_VER_MMP}~snapshot${GTSAM_SNAPSHOT_VERSION}${DEBIAN_DIST}/debian
	#cp ${GTSAM_EXTERN_DEBIAN_DIR}/changelog changelog
	cp /tmp/my_changelog changelog
	DEBCHANGE_CMD="--newversion ${GTSAM_VERSION_STR}~snapshot${GTSAM_SNAPSHOT_VERSION}${DEBIAN_DIST}-1"
	echo "Changing to a new Debian version: ${DEBCHANGE_CMD}"
	echo "Adding a new entry to debian/changelog for distribution ${DEBIAN_DIST}"
	DEBEMAIL="Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD -b --distribution ${DEBIAN_DIST} --force-distribution New version of upstream sources.

	cp changelog /tmp/my_changelog

	echo "Now, let's build the source Deb package with 'debuild -S -sa':"
	cd ..
	# -S: source package
	# -sa: force inclusion of sources
	# -d: don't check dependencies in this system
	debuild -S -sa -d

	# Make a copy of all these packages:
	cd ..
	mkdir -p $gtsam_ubuntu_OUT_DIR/$DEBIAN_DIST
	cp gtsam_* $gtsam_ubuntu_OUT_DIR/${DEBIAN_DIST}/
	echo ">>>>>> Saving packages to: $gtsam_ubuntu_OUT_DIR/$DEBIAN_DIST/"
done


exit 0
