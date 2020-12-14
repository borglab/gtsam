# Instructions

Build all docker images, in order:

```bash
(cd ubuntu-boost-tbb && ./build.sh)
(cd ubuntu-gtsam && ./build.sh)
(cd ubuntu-gtsam-python && ./build.sh)
(cd ubuntu-gtsam-python-vnc && ./build.sh)
```

Then launch with: 

    docker run -p 5900:5900 dellaert/ubuntu-gtsam-python-vnc:bionic

Then open a remote VNC X client, for example:

    sudo apt-get install tigervnc-viewer
    xtigervncviewer :5900


