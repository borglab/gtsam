# Get the base Ubuntu/GTSAM image from Docker Hub
FROM dellaert/ubuntu-gtsam:bionic

# Install pip
RUN apt-get install -y python-pip python-dev

# Install python wrapper requirements
RUN pip install -r /usr/src/gtsam/cython/requirements.txt

# Run cmake again, now with cython toolbox on
WORKDIR /usr/src/gtsam/build
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON \
    ..

# Build again, as ubuntu-gtsam image cleaned
RUN make -j3 install && make clean

# Needed to run python wrapper:
RUN echo 'export PYTHONPATH=/usr/local/cython/' >> /root/.bashrc 

# Run bash
CMD ["bash"]
