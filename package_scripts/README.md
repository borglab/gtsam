
# How to generate Debian packages

    cd [GTSAM_SOURCE_ROOT]
    bash package_scripts/prepare_debian.sh


# How to generate Ubuntu packages for a PPA

    cd [GTSAM_SOURCE_ROOT]
    bash package_scripts/prepare_ubuntu_pkgs_for_ppa.sh
    cd ~/gtsam_ubuntu
    bash [GTSAM_SOURCE_ROOT]/upload_all_gtsam_ppa.sh

