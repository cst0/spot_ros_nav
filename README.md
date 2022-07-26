# spot_ros_nav
Configurations and launchfiles to integrate move_base with the Spot. Currently in beta, there will be breaking changes.

Assuming that:

1) A spot wrapper is providing odometry transforms and the ability to specify commands on the /cmd_vel topic
2) external sensing is providing data about the environment on the /scan topic (for example, as provided by the Velodyne in the EAP)

this package will provide the ability to:

1) record a map of the environment for later use
2) localize within an existing map, allowing for obstacle-avoidant planningand navigation

## Usage (non-docker)
Run the `navigation.launch` to perform basic navigation. The default configurations likley meet your needs but can be adapted.

## Usage (docker)
For deployment convenience, a set of docker-based functionality is provided. It's designed such that shared volumes can be used to quickly change the configuration parameters without needing to re-build anything. 
