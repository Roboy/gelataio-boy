cmake_minimum_required(VERSION 2.8.3)
project(coordinator)

find_package(catkin REQUIRED COMPONENTS  rospy message_generation std_msgs roboy_control_msgs)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs roboy_control_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/luigi_scoop_server.py
  scripts/fake_luigi_client.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
)
install(FILES
  launch/start.launch
  launch/fake_luigi.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
