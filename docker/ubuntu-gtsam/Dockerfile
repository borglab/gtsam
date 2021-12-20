# Ubuntu image with GTSAM installed. Configured with  Boost and TBB support.

# Get the base Ubuntu image from Docker Hub
FROM dellaert/ubuntu-boost-tbb:bionic

# Install git
RUN apt-get update && \
    apt-get install -y git

# Install compiler
RUN apt-get install -y build-essential

# Clone GTSAM (develop branch)
WORKDIR /usr/src/
RUN git clone --single-branch --branch develop https://github.com/borglab/gtsam.git

# Change to build directory. Will be created automatically.
WORKDIR /usr/src/gtsam/build
# Run cmake
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=OFF \
    ..

# Build
RUN make -j4 install && make clean

# Needed to link with GTSAM
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:LD_LIBRARY_PATH' >> /root/.bashrc

# Run bash
CMD ["bash"]
