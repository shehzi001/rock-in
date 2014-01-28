# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()


if(basic_navigation_CONFIG_INCLUDED)
  return()
endif()
set(basic_navigation_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(basic_navigation_SOURCE_PREFIX /home/eren/ros/hydro/youbot_ws/src/binary_bitbots/src/basic_navigation)
  set(basic_navigation_DEVEL_PREFIX /home/eren/ros/hydro/youbot_ws/src/binary_bitbots/devel)
  set(basic_navigation_INSTALL_PREFIX "")
  set(basic_navigation_PREFIX ${basic_navigation_DEVEL_PREFIX})
else()
  set(basic_navigation_SOURCE_PREFIX "")
  set(basic_navigation_DEVEL_PREFIX "")
  set(basic_navigation_INSTALL_PREFIX /home/eren/ros/hydro/youbot_ws/src/binary_bitbots/install)
  set(basic_navigation_PREFIX ${basic_navigation_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'basic_navigation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(basic_navigation_FOUND_CATKIN_PROJECT TRUE)

if(NOT "" STREQUAL "")
  set(basic_navigation_INCLUDE_DIRS "")
  set(_include_dirs "")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir}" STREQUAL "include")
      get_filename_component(include "${basic_navigation_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'basic_navigation' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'eren <eren@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'basic_navigation' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/eren/ros/hydro/youbot_ws/src/binary_bitbots/src/basic_navigation/${idir}'.  Ask the maintainer 'eren <eren@todo.todo>' to fix it.")
    endif()
    _list_append_unique(basic_navigation_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^debug|optimized|general$")
    list(APPEND basic_navigation_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND basic_navigation_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND basic_navigation_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/eren/ros/hydro/youbot_ws/src/binary_bitbots/devel/lib;/home/eren/ros/hydro/youbot_ws/devel/lib;/home/eren/ros/hydro/rockin_ws/devel/lib;/home/eren/ros/hydro/sdp_ws/devel/lib;/home/eren/ros/hydro/catkin_ws/devel/lib;/opt/ros/hydro/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(basic_navigation_LIBRARY_DIRS ${lib_path})
      list(APPEND basic_navigation_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'basic_navigation'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND basic_navigation_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(basic_navigation_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${basic_navigation_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 basic_navigation_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${basic_navigation_dep}_FOUND)
      find_package(${basic_navigation_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${basic_navigation_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(basic_navigation_INCLUDE_DIRS ${${basic_navigation_dep}_INCLUDE_DIRS})
  _list_append_deduplicate(basic_navigation_LIBRARIES ${${basic_navigation_dep}_LIBRARIES})
  _list_append_unique(basic_navigation_LIBRARY_DIRS ${${basic_navigation_dep}_LIBRARY_DIRS})
  list(APPEND basic_navigation_EXPORTED_TARGETS ${${basic_navigation_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${basic_navigation_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
