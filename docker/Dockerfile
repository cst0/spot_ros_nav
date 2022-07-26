FROM ros:noetic

RUN apt update
RUN apt install -y git

RUN git clone https://github.com/cst0/spot_ros_nav.git /overlay_ws/src/spot_ros_nav/
WORKDIR /overlay_ws
RUN rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r -y
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'

WORKDIR /
RUN mkdir /config

RUN touch /custom_entrypoint.sh
RUN echo "#!/bin/bash"					 >> /custom_entrypoint.sh
RUN echo "set -e" 					 >> /custom_entrypoint.sh
RUN echo "# setup ros environment"			 >> /custom_entrypoint.sh
RUN echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\" --" >> /custom_entrypoint.sh
RUN echo "source \"/overlay_ws/devel/setup.bash\" --"    >> /custom_entrypoint.sh
RUN echo "exec \"\$@\""				 >> /custom_entrypoint.sh
#RUN echo "roslaunch spot_ros_nav docker_navigate.launch" >> /custom_entrypoint.sh

RUN cat /custom_entrypoint.sh
RUN chmod +x /custom_entrypoint.sh

ENTRYPOINT ["/custom_entrypoint.sh"]
