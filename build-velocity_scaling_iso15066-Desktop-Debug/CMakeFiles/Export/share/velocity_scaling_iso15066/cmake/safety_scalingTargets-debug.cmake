#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "safety_scaling::velocity_scaling" for configuration "Debug"
set_property(TARGET safety_scaling::velocity_scaling APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(safety_scaling::velocity_scaling PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libvelocity_scaling.so"
  IMPORTED_SONAME_DEBUG "libvelocity_scaling.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS safety_scaling::velocity_scaling )
list(APPEND _IMPORT_CHECK_FILES_FOR_safety_scaling::velocity_scaling "${_IMPORT_PREFIX}/lib/libvelocity_scaling.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
