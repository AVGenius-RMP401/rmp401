cmake_minimum_required(VERSION 3.8)
project(rmp401_description)

find_package(ament_cmake REQUIRED)

# 리소스 설치
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
