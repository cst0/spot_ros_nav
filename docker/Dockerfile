FROM ros:noetic

RUN apt update
RUN apt install -y git

COPY . /overlay_ws/src
WORKDIR /overlay_ws
RUN rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r -y
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'
RUN mkdir /config

RUN echo "/ros_entrypoint.sh                                   \
            /bin/bash -c '                                     \
            source /overlay_ws/devel/setup.bash              &&\
            roslaunch spot_ros_nav docker_navigate.launch'"    \
            > custom_ros_entry.sh

RUN chmod +x custom_ros_entry.sh
ENTRYPOINT ["/custom_ros_entry.sh"]
