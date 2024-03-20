#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rsl::rsl" for configuration ""
set_property(TARGET rsl::rsl APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rsl::rsl PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librsl.so"
  IMPORTED_SONAME_NOCONFIG "librsl.so"
  )

list(APPEND _cmake_import_check_targets rsl::rsl )
list(APPEND _cmake_import_check_files_for_rsl::rsl "${_IMPORT_PREFIX}/lib/librsl.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
