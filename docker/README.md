# Instructions

Build all docker images, in order:

```bash
(cd ubuntu-boost-tbb && exec build.sh)
(cd ubuntu-gtsam && exec build.sh)
(cd ubuntu-gtsam-python && exec build.sh)
(cd ubuntu-gtsam-python-vnc && exec build.sh)
```

Then launch with: 

    docker run dellaert/ubuntu-gtsam-python-vnc:bionic


