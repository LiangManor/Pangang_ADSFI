# Install script for directory: /home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin" TYPE EXECUTABLE FILES "/home/inwinic/Wll_project/CRRC/PanG/ADSFI/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/planning/planning")
  if(EXISTS "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning"
         OLD_RPATH "/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/lib/mdc_vector:/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/lib/mdc/base-plat:/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/local/Ascend/runtime/lib64/stub:/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/local/Ascend/runtime/lib64:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/bin/llvm-strip" "$ENV{DESTDIR}/home/inwinic/Wll_project/CRRC/PanG/ADSFI/modules/planning/bin/planning")
    endif()
  endif()
endif()

