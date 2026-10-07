#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robot_description::robot_description" for configuration ""
set_property(TARGET robot_description::robot_description APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robot_description::robot_description PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librobot_description.so"
  IMPORTED_SONAME_NOCONFIG "librobot_description.so"
  )

list(APPEND _cmake_import_check_targets robot_description::robot_description )
list(APPEND _cmake_import_check_files_for_robot_description::robot_description "${_IMPORT_PREFIX}/lib/librobot_description.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
