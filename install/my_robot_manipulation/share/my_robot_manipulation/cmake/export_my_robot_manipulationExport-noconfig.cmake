#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_robot_manipulation::arm_ik" for configuration ""
set_property(TARGET my_robot_manipulation::arm_ik APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(my_robot_manipulation::arm_ik PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libarm_ik.so"
  IMPORTED_SONAME_NOCONFIG "libarm_ik.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS my_robot_manipulation::arm_ik )
list(APPEND _IMPORT_CHECK_FILES_FOR_my_robot_manipulation::arm_ik "${_IMPORT_PREFIX}/lib/libarm_ik.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
