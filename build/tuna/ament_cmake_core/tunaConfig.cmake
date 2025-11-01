# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tuna_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tuna_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tuna_FOUND FALSE)
  elseif(NOT tuna_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tuna_FOUND FALSE)
  endif()
  return()
endif()
set(_tuna_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tuna_FIND_QUIETLY)
  message(STATUS "Found tuna: 0.0.0 (${tuna_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tuna' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tuna_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tuna_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tuna_DIR}/${_extra}")
endforeach()
