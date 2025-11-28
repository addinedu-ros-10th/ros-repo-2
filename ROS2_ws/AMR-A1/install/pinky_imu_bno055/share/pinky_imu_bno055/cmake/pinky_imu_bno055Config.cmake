# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pinky_imu_bno055_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pinky_imu_bno055_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pinky_imu_bno055_FOUND FALSE)
  elseif(NOT pinky_imu_bno055_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pinky_imu_bno055_FOUND FALSE)
  endif()
  return()
endif()
set(_pinky_imu_bno055_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pinky_imu_bno055_FIND_QUIETLY)
  message(STATUS "Found pinky_imu_bno055: 0.0.0 (${pinky_imu_bno055_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pinky_imu_bno055' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT pinky_imu_bno055_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pinky_imu_bno055_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pinky_imu_bno055_DIR}/${_extra}")
endforeach()
