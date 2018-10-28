# Get the base Ubuntu image from Docker Hub
FROM dellaert/ubuntu-boost-tbb-eigen3:bionic

# Install git
RUN apt-get update && \ 
    apt-get install -y git

# Install compiler
RUN apt-get install -y build-essential

# Clone GTSAM
WORKDIR /usr/src/
RUN git clone https://bitbucket.org/gtborg/gtsam.git

# Run cmake
RUN mkdir /usr/src/gtsam/build
WORKDIR /usr/src/gtsam/build
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=OFF \
    ..

# Build
RUN make -j3 install && make clean

# Needed to link with GTSAM
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib' >> /root/.bashrc 

# Run bash
CMD ["bash"]
