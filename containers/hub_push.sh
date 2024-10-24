#!/usr/bin/env bash

# A script to push images to Docker Hub

declare -a ubuntu_tags=("22.04")
declare -a gtsam_git_tags=("4.2.0")
declare -a gtsam_with_tbb_options=("OFF" "ON")
declare -a gtsam_build_python_options=("OFF" "ON")

for ubuntu_tag in "${ubuntu_tags[@]}"; do
for gtsam_git_tag in "${gtsam_git_tags[@]}"; do
for gtsam_with_tbb in "${gtsam_with_tbb_options[@]}"; do
for gtsam_build_python in "${gtsam_build_python_options[@]}"; do

    touch .env
    echo "UBUNTU_TAG=${ubuntu_tag}" > .env
    echo "GTSAM_GIT_TAG=${gtsam_git_tag}" >> .env
    echo "GTSAM_WITH_TBB=${gtsam_with_tbb}" >> .env
    echo "GTSAM_BUILD_PYTHON=${gtsam_build_python}" >> .env
    echo "CORES=4" >> .env

    docker compose build

    docker tag gtsam:"${gtsam_git_tag}-tbb-${gtsam_with_tbb}-python-${gtsam_build_python}_${ubuntu_tag}" \
        docker.io/borglab/gtsam:"${gtsam_git_tag}-tbb-${gtsam_with_tbb}-python-${gtsam_build_python}_${ubuntu_tag}"

    docker push docker.io/borglab/gtsam:"${gtsam_git_tag}-tbb-${gtsam_with_tbb}-python-${gtsam_build_python}_${ubuntu_tag}"

done
done
done
done
