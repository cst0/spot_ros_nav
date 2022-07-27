#!/bin/bash

/bin/bash -c 'cd .. && 				\
		docker run -it --rm --privileged --net=host  	\
		    --volume '/home/cst/ws_spot/src/spot_ros_nav/config:/config:ro'\
		    --volume '/home/cst/ws_spot/src/spot_ros_nav/maps:/maps:rw'    \
		    --volume '/tmp/.X11-unix:/tmp/.X11-unix:rw' \
		    --volume '/etc/localtime:/etc/localtime:ro' \
		    spot_ros_nav /bin/bash'
