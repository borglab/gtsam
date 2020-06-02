# Get the base Ubuntu image from Docker Hub
FROM ubuntu:bionic

# Update apps on the base image
RUN apt-get -y update && apt install -y

# Install C++
RUN apt-get -y install build-essential 

# Install boost and cmake
RUN apt-get -y install libboost-all-dev cmake

# Install TBB
RUN apt-get -y install libtbb-dev
