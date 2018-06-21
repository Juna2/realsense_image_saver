cmake_minimum_required(VERSION 2.8.3)
project(realsense_image_saver)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing

catkin_install_python(PROGRAMS
  src/image_saver.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
catkin_install_python(PROGRAMS
  src/ur_robot.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
install(FILES
  src/image_saver.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
install(FILES
  src/ur_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)


