#!/bin/bash

/bin/bash -c '                          \
    cd ..                             &&\
    set -e                            &&\
    docker build . -t spot_ros_nav'
