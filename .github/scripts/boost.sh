### Script to install Boost
BOOST_FOLDER=boost_${BOOST_VERSION//./_}

# Download Boost
wget https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/${BOOST_FOLDER}.tar.gz

# Unzip
tar -zxf ${BOOST_FOLDER}.tar.gz

# Bootstrap
cd ${BOOST_FOLDER}/
./bootstrap.sh

# Build and install
sudo ./b2 install

# Rebuild ld cache
sudo ldconfig
