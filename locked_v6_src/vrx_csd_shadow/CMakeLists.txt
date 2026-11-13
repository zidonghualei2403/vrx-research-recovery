cmake_minimum_required(VERSION 3.10)
project(vrx_csd_shadow)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(CsdOceanCurrent SHARED src/CsdOceanCurrent.cc)
target_link_libraries(CsdOceanCurrent PUBLIC
  gz-sim8::core
  gz-plugin2::gz-plugin2
)

install(TARGETS CsdOceanCurrent DESTINATION lib)

install(PROGRAMS
  scripts/csd_shadow_graph.py
  scripts/analyze_shadow_csv.py
  scripts/two_vessel_crossing_driver.py
  scripts/six_vessel_crossing_driver.py
  scripts/spawn_model_retry.py
  scripts/replay_historical_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
