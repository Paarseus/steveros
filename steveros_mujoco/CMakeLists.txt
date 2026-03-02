cmake_minimum_required(VERSION 3.8)
project(steveros_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config mjcf
  DESTINATION share/${PROJECT_NAME}
)

# Symlink steveros_description meshes into mjcf/ so the MJCF compiler can find them.
# colcon uses per-package install prefixes, so we navigate up to the workspace install root.
install(CODE "
  execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    \${CMAKE_INSTALL_PREFIX}/../steveros_description/share/steveros_description/meshes
    \${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/mjcf/meshes
  )
")

ament_package()
