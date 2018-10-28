# Get the base Ubuntu/GTSAM image from Docker Hub
FROM dellaert/ubuntu-gtsam-python:bionic

# Things needed to get a python GUI
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y python-tk
RUN pip install matplotlib

# Install a VNC X-server, Frame buffer, and windows manager
RUN apt-get install -y x11vnc xvfb fluxbox

# Finally, install wmctrl needed for bootstrap script
RUN apt-get install -y wmctrl 

# Copy bootstrap script and make sure it runs
COPY bootstrap.sh /

CMD '/bootstrap.sh'
