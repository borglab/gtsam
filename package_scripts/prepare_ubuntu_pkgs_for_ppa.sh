#!/bin/bash
# Creates a set of packages for each different Ubuntu distribution, with the
# intention of uploading them to:
#   https://launchpad.net/~joseluisblancoc/+archive/mrpt
#
# JLBC, 2010
# [Addition 2012:]
#
# You can declare a variable (in the caller shell) with extra flags for the
# CMake in the final ./configure like:
#  MRPT_PKG_CUSTOM_CMAKE_PARAMS="\"-DDISABLE_SSE3=ON\""
#

set -e

# List of distributions to create PPA packages for:
LST_DISTROS=(xenial bionic cosmic disco)


# Checks
# --------------------------------
if [ -f version_prefix.txt ];
then
	MRPT_VERSION_STR=`head -n 1 version_prefix.txt`
	MRPT_VERSION_MAJOR=${MRPT_VERSION_STR:0:1}
	MRPT_VERSION_MINOR=${MRPT_VERSION_STR:2:1}
	MRPT_VERSION_PATCH=${MRPT_VERSION_STR:4:1}

	MRPT_VER_MM="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}"
	MRPT_VER_MMP="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}.${MRPT_VERSION_PATCH}"
	echo "MRPT version: ${MRPT_VER_MMP}"
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

if [ -z "${MRPT_UBUNTU_OUT_DIR}" ]; then
       export MRPT_UBUNTU_OUT_DIR="$HOME/mrpt_ubuntu"
fi
MRPTSRC=`pwd`
if [ -z "${MRPT_DEB_DIR}" ]; then
       export MRPT_DEB_DIR="$HOME/mrpt_debian"
fi
MRPT_EXTERN_DEBIAN_DIR="$MRPTSRC/packaging/debian/"
EMAIL4DEB="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>"

# Clean out dirs:
rm -fr $MRPT_UBUNTU_OUT_DIR/

# -------------------------------------------------------------------
# And now create the custom packages for each Ubuntu distribution:
# -------------------------------------------------------------------
# Xenial:armhf does not have any version of liboctomap-dev:
export MRPT_RELEASE_EXTRA_OTHERLIBS_URL="https://github.com/MRPT/octomap/archive/devel.zip"
export MRPT_RELEASE_EXTRA_OTHERLIBS_PATH="otherlibs/octomap.zip"

count=${#LST_DISTROS[@]}
IDXS=$(seq 0 $(expr $count - 1))

cp ${MRPT_EXTERN_DEBIAN_DIR}/changelog /tmp/my_changelog

for IDX in ${IDXS};
do
	DEBIAN_DIST=${LST_DISTROS[$IDX]}

	# -------------------------------------------------------------------
	# Call the standard "prepare_debian.sh" script:
	# -------------------------------------------------------------------
	cd ${MRPTSRC}
	bash scripts/prepare_debian.sh -s -u -h -d ${DEBIAN_DIST} ${EMBED_EIGEN_FLAG}  -c "${MRPT_PKG_CUSTOM_CMAKE_PARAMS}"

	CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	source $CUR_SCRIPT_DIR/prepare_debian_gen_snapshot_version.sh # populate MRPT_SNAPSHOT_VERSION

	echo "===== Distribution: ${DEBIAN_DIST}  ========="
	cd ${MRPT_DEB_DIR}/mrpt-${MRPT_VER_MMP}~snapshot${MRPT_SNAPSHOT_VERSION}${DEBIAN_DIST}/debian
	#cp ${MRPT_EXTERN_DEBIAN_DIR}/changelog changelog
	cp /tmp/my_changelog changelog
	DEBCHANGE_CMD="--newversion 1:${MRPT_VERSION_STR}~snapshot${MRPT_SNAPSHOT_VERSION}${DEBIAN_DIST}-1~ppa1~${DEBIAN_DIST}"
	echo "Changing to a new Debian version: ${DEBCHANGE_CMD}"
	echo "Adding a new entry to debian/changelog for distribution ${DEBIAN_DIST}"
	DEBEMAIL=${EMAIL4DEB} debchange $DEBCHANGE_CMD -b --distribution ${DEBIAN_DIST} --force-distribution New version of upstream sources.

	cp changelog /tmp/my_changelog

	echo "Now, let's build the source Deb package with 'debuild -S -sa':"
	cd ..
	# -S: source package
	# -sa: force inclusion of sources
	# -d: don't check dependencies in this system
	debuild -S -sa -d

	# Make a copy of all these packages:
	cd ..
	mkdir -p $MRPT_UBUNTU_OUT_DIR/$DEBIAN_DIST
	cp mrpt_* $MRPT_UBUNTU_OUT_DIR/$DEBIAN_DIST/
	echo ">>>>>> Saving packages to: $MRPT_UBUNTU_OUT_DIR/$DEBIAN_DIST/"
done


exit 0
