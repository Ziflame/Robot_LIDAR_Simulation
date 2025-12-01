cmake_minimum_required(VERSION 3.3)
project(TestOpenCV)

set(CMAKE_CXX_STANDARD_REQUIRED 11)

find_package(
    OpenCV REQUIRED
)

include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
)

set(SOURCES
    src/Map.cpp
    src/Robot.cpp
    src/Simulation.cpp
    src/Lidar.cpp
    src/OccupancyGrid.cpp
    src/BehaviorManager.cpp
    src/ArucoManager.cpp
    src/main.cpp
    
    
)

set(HEADERS
    include/Map.hpp
    include/Robot.hpp
    include/Simulation.hpp
    include/Lidar.hpp
    include/OccupancyGrid.hpp
    include/BehaviorManager.hpp
    include/ArucoManager.hpp
)
    

set(CMAKE_BUILD_TYPE Debug) 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")  

add_executable(main ${SOURCES})
target_link_libraries(main ${OpenCV_LIBS} )

