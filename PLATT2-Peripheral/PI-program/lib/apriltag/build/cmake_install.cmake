# Install script for directory: /mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/aarch64-linux-gnu-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/apriltag.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/apriltag_math.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/apriltag_pose.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco4x4_100.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco4x4_1000.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco4x4_250.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco4x4_50.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco5x5_100.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco5x5_1000.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco5x5_250.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco5x5_50.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco6x6_100.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco6x6_1000.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco6x6_250.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco6x6_50.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco7x7_100.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco7x7_1000.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco7x7_250.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagAruco7x7_50.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/aruco" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/aruco/tagArucoMIP36h12.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/debug_print.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/doubles.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/doubles_floats_impl.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/floats.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/g2d.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/getopt.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/homography.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/image_types.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/image_u8.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/image_u8_parallel.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/image_u8x3.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/image_u8x4.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/matd.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/math_util.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/pam.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/pjpeg.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/pnm.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/postscript_utils.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/pthreads_cross.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/string_util.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/svd22.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/time_util.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/timeprofile.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/unionfind.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/workerpool.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/zarray.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/zhash.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag/common" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/common/zmaxheap.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tag16h5.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tag25h9.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tag36h10.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tag36h11.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tagCircle21h7.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tagCircle49h12.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tagCustom48h12.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tagStandard41h12.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/apriltag" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/tagStandard52h13.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/libapriltag.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake/apriltagTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake/apriltagTargets.cmake"
         "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/CMakeFiles/Export/092e78f427159fbe8f9640b94fd1abbe/apriltagTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake/apriltagTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake/apriltagTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/CMakeFiles/Export/092e78f427159fbe8f9640b94fd1abbe/apriltagTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/CMakeFiles/Export/092e78f427159fbe8f9640b94fd1abbe/apriltagTargets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/apriltag/cmake" TYPE FILE FILES
    "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/generated/apriltagConfig.cmake"
    "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/generated/apriltagConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/apriltag.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/apriltag_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/apriltag_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/apriltag_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/apriltag_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/apriltag_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/apriltag_demo")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/aarch64-linux-gnu-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/apriltag_demo")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/mnt/c/Users/logan/Documents/GitHub/PLATT2-Override-Software/PLATT2-Peripheral/PI-program/lib/apriltag/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
