# How to build a GTSAM debian package

To use the ``debuild`` command, install the ``devscripts`` package

    sudo apt install devscripts

Change into the gtsam directory, then run:

    debuild -us -uc -j4

Adjust the ``-j4`` depending on how many CPUs you want to build on in
parallel. 
