#!/bin/bash

/bin/bash -c '                          \
    cd ..                             &&\
    set -e                            &&\
    docker build ./docker -t spot_ros_nav'
