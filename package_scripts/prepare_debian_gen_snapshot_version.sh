#!/bin/bash

# See https://reproducible-builds.org/specs/source-date-epoch/
# get SOURCE_DATE_EPOCH with UNIX time_t
if [ -d ".git" ];
then
  SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
  echo "Error: intended for use from within a git repository"
  exit 1
fi
GTSAM_SNAPSHOT_VERSION=$(date -d @$SOURCE_DATE_EPOCH +%Y%m%d-%H%M)

GTSAM_SNAPSHOT_VERSION+="-git-"
GTSAM_SNAPSHOT_VERSION+=`git rev-parse --short=8 HEAD`
GTSAM_SNAPSHOT_VERSION+="-"

# x.y.z version components:
GTSAM_VERSION_MAJOR=$(grep "(GTSAM_VERSION_MAJOR" CMakeLists.txt | sed -r 's/^.*GTSAM_VERSION_MAJOR\s*([0-9])*.*$/\1/g')
GTSAM_VERSION_MINOR=$(grep "(GTSAM_VERSION_MINOR" CMakeLists.txt | sed -r 's/^.*GTSAM_VERSION_MINOR\s*([0-9])*.*$/\1/g')
GTSAM_VERSION_PATCH=$(grep "(GTSAM_VERSION_PATCH" CMakeLists.txt | sed -r 's/^.*GTSAM_VERSION_PATCH\s*([0-9])*.*$/\1/g')

GTSAM_VER_MM="${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}"
GTSAM_VER_MMP="${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}"
GTSAM_VERSION_STR=$GTSAM_VER_MMP
