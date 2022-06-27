cmake_minimum_required(VERSION 3.5)
project(flex_nav_turtlebot2_bringup)

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
###########
## Build ##
###########


#############
## Install ##
#############


# Mark anything (useful) else for installation
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY paths
        DESTINATION share/${PROJECT_NAME}
)

# Install bash launch scripts
install(PROGRAMS
  scripts/hw-tmux
  scripts/hw-kinect-tmux
  scripts/launch-ocs
  scripts/launch-onboard
  scripts/launch-sim
  scripts/log-flexbe
  scripts/log-flexbe-commands
  scripts/ocs-tmux
  scripts/onboard-tmux
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
