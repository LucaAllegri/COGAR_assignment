# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_create_scene_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED create_scene_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(create_scene_FOUND FALSE)
  elseif(NOT create_scene_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(create_scene_FOUND FALSE)
  endif()
  return()
endif()
set(_create_scene_CONFIG_INCLUDED TRUE)

# output package information
if(NOT create_scene_FIND_QUIETLY)
  message(STATUS "Found create_scene: 0.0.0 (${create_scene_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'create_scene' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${create_scene_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(create_scene_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${create_scene_DIR}/${_extra}")
endforeach()
