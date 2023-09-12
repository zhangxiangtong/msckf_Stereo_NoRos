cmake_minimum_required(VERSION 2.8.12)
project(msckf_vio CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive -O3")
# Modify cmake module path if new .cmake files are required
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Pangolin REQUIRED)
message("Pangolin_INCLUDE_DIRS : "${Pangolin_INCLUDE_DIRS})
message("Pangolin_LIBRARIES : "${Pangolin_LIBRARIES})

include_directories(
                    ${EIGEN3_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/include/msckf_vio
                    ${PROJECT_SOURCE_DIR}/include/DUtils
                    ${PROJECT_SOURCE_DIR}/include/display
                  )

# Msckf Vio
add_executable( msckf_vio
                src/System.cpp
                src/msckf_vio/msckf_vio.cpp
                src/msckf_vio/utils.cpp
                src/msckf_vio/image_processor.cpp
                src/DUtils/Timestamp.cpp
                src/DUtils/Random.cpp
                src/msckf_stereo.cpp
                src/display/Viewer.cc
                src/display/VioDrawer.cc
               )

target_link_libraries(msckf_vio
  ${SUITESPARSE_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  -lboost_serialization
)
