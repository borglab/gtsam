# After running this script, connect VNC client to 0.0.0.0:5900
docker run -it \
    --workdir="/usr/src/gtsam" \
    -p 5900:5900 \
    borglab/ubuntu-gtsam-python-vnc:bionic