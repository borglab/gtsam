#  GTSAM Ubuntu image with Python wrapper support.

# Get the base Ubuntu/GTSAM image from Docker Hub
FROM borglab/ubuntu-gtsam:bionic

# Install pip
RUN apt-get install -y python3-pip python3-dev

# Run cmake again, now with python toolbox on
WORKDIR /usr/src/gtsam/build
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_PYTHON_VERSION=3\
    ..

# Build again, as ubuntu-gtsam image cleaned
RUN make -j4 install
RUN make python-install
RUN make clean

# Needed to run python wrapper:
RUN echo 'export PYTHONPATH=/usr/local/python/:$PYTHONPATH' >> /root/.bashrc

# Run bash
CMD ["bash"]
