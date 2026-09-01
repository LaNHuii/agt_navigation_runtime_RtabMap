cmake_minimum_required(VERSION 3.8)
project(agt_rtabmap_backend)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/rtabmap_adapter.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rtabmap_adapter test/test_rtabmap_adapter.py)
endif()

ament_package()
