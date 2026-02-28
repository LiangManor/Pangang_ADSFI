# Install script for directory: /home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/3dBbs_lib/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/camera_det/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/innovusion_lib/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/ins_abstract/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_A3/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_A3_192/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_A4/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_A4_192/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_A4_innovusion/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_B1/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_B1_192/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_B2/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_B2_192/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_B2_innovusion/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_abstract_B3_innovusion/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_det_cluster/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_slam/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/lidar_tracker/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/location/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/multisensor_fusion/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/planning/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/radar_track_perception/cmake_install.cmake")
  include("/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/traffic_light_det/cmake_install.cmake")

endif()

