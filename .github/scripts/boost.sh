### Script to install Boost
set -e

BOOST_FOLDER=boost_${BOOST_VERSION//./_}

# Download Boost
wget https://archives.boost.io/release/${BOOST_VERSION}/source/${BOOST_FOLDER}.tar.gz

# Unzip
tar -zxf ${BOOST_FOLDER}.tar.gz

# Bootstrap
cd ${BOOST_FOLDER}/
./bootstrap.sh --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex

# Build and install
sudo ./b2 -j$(nproc) install

# Rebuild ld cache
sudo ldconfig
