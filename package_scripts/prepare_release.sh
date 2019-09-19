#!/bin/bash
# Export sources from a git tree and prepare it for a public release.
# Jose Luis Blanco Claraco, 2019 (for GTSAM)
# Jose Luis Blanco Claraco, 2008-2018 (for MRPT)

set -e  # exit on error
#set -x  # for debugging

# Checks
# --------------------------------
if [ -f version_prefix.txt ];
then
	if [ -z ${GTSAM_VERSION_STR+x} ];
	then
		source package_scripts/prepare_debian_gen_snapshot_version.sh
	fi
	echo "ERROR: Run this script from the GTSAM source tree root directory."
	exit 1
fi

GTSAM_SRC=`pwd`
OUT_RELEASES_DIR="$HOME/gtsam_release"

OUT_DIR=$OUT_RELEASES_DIR/gtsam-${GTSAM_VERSION_STR}

echo "=========== Generating GTSAM release ${GTSAM_VER_MMP} =================="
echo "GTSAM_VERSION_STR   : ${GTSAM_VERSION_STR}"
echo "OUT_DIR            : ${OUT_DIR}"
echo "============================================================"
echo

# Prepare output directory:
rm -fR $OUT_RELEASES_DIR  || true
mkdir -p ${OUT_DIR}

# Export / copy sources to target dir:
if [ -d "$GTSAM_SRC/.git" ];
then
	echo "# Exporting git source tree to ${OUT_DIR}"
	git archive --format=tar HEAD | tar -x -C ${OUT_DIR}

	# Remove VCS control files:
	find ${OUT_DIR} -name '.gitignore' | xargs rm

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
	echo "# Copying sources to ${OUT_DIR}"
	cp -R . ${OUT_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(date +%s)
fi

# See https://reproducible-builds.org/specs/source-date-epoch/
echo $SOURCE_DATE_EPOCH > ${OUT_DIR}/SOURCE_DATE_EPOCH

cd ${OUT_DIR}

# Dont include Debian files in releases:
rm -fR package_scripts

# Orig tarball:
cd ..
echo "# Creating orig tarball: gtsam-${GTSAM_VERSION_STR}.tar.gz"
tar czf gtsam-${GTSAM_VERSION_STR}.tar.gz gtsam-${GTSAM_VERSION_STR}

rm -fr gtsam-${GTSAM_VERSION_STR}

# GPG signature:
gpg --armor --detach-sign gtsam-${GTSAM_VERSION_STR}.tar.gz
