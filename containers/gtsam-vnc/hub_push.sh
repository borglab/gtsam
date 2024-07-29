#!/usr/bin/env bash

# A script to push images to Docker Hub

declare -a gtsam_tags=("4.2.0-tbb-ON-python-ON_22.04")

for gtsam_tag in "${gtsam_tags[@]}"; do

    touch gtsam-vnc/.env
    echo "GTSAM_TAG=${gtsam_tag}" > gtsam-vnc/.env

    docker compose --file gtsam-vnc/compose.yaml build

    docker tag gtsam-vnc:"${gtsam_tag}" \
        docker.io/borglab/gtsam-vnc:"${gtsam_tag}"

    docker push docker.io/borglab/gtsam-vnc:"${gtsam_tag}"

done
