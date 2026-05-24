cmake_minimum_required(VERSION 3.8)
project(zyz_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/arm.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN solver::arm::ZYZIK
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE

)
