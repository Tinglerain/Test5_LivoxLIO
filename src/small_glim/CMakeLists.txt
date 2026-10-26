cmake_minimum_required(VERSION 3.10)
project(small_glim)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${sources})

find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

find_package(PCL REQUIRED)
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

find_package(GTSAM 4.3 REQUIRED)
target_link_libraries(${PROJECT_NAME} gtsam)

find_package(gtsam_points 1.2.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} gtsam_points::gtsam_points)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_glim::SmallGlimNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# 如果gtsam_points或GTSAM构建时没有使用-march=native，则这里不要使用-march=native
# 同理如果gtsam_points或GTSAM构建时没有开启ASAN，则这里也不要使用-fsanitize=address
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Debug>:-O0 -g -Wall -Wextra -Wpedantic -Wconversion -fsanitize=undefined -fstack-protector-strong -fcf-protection=full>
  $<$<CONFIG:Release>:-O3 -DNDEBUG -Wall -Wextra -Wpedantic -Wconversion -flto=auto>
)

target_link_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Debug>:-fsanitize=undefined>
  $<$<CONFIG:Release>:-flto=auto>
)

target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<CONFIG:Debug>:-fsanitize=undefined>)
target_link_options(${PROJECT_NAME}_node PRIVATE $<$<CONFIG:Debug>:-fsanitize=undefined>)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cloud_ops test/test_cloud_ops.cpp)
  target_link_libraries(test_cloud_ops ${PROJECT_NAME})
  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_link_libraries(test_pose_utils ${PROJECT_NAME})
  ament_add_gtest(test_initial_attitude test/test_initial_attitude.cpp)
  target_link_libraries(test_initial_attitude ${PROJECT_NAME})
  ament_add_gtest(test_deskew test/test_deskew.cpp)
  target_link_libraries(test_deskew ${PROJECT_NAME})
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)