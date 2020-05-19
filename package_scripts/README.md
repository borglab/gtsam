# How to build Debian and Ubuntu Packages

## Preparations

Packages must be signed with a GPG key. First have a look of the keys
you have available:

    gpg --list-secret-keys

If you don't have one, create one, then list again.

Pick a secret key you like from the listed keys, for instance
"Your Name <your.email@yourprovider.com>". Then unlock that key by
signing a dummy file. The following line should pop up a window to
enter the passphrase:

    echo | gpg --local-user "Your Name <your.email@yourprovider.com>" -s >/dev/null

Now you can run the below scripts. Without this step they will fail
with "No secret key" or similar messages.

## How to generate a Debian package

Run the package script, providing a name/email that matches your PGP key.

    cd [GTSAM_SOURCE_ROOT]
    bash package_scripts/prepare_debian.sh -e "Your Name <your.email@yourprovider.com>"


## How to generate Ubuntu packages for a PPA

Run the packaging script, passing the name of the gpg key
(see above) with the "-e" option:

    cd [GTSAM_SOURCE_ROOT]
    bash package_scripts/prepare_ubuntu_pkgs_for_ppa.sh -e "Your Name <your.email@yourprovider.com>"

Check that you have uploaded this key to the ubuntu key server, and
have added the key to your account.

Upload the package to your ppa:

    cd ~/gtsam_ubuntu
    bash [GTSAM_SOURCE_ROOT]/package_scripts/upload_all_gtsam_ppa.sh -p "ppa:your-name/ppa-name"
