#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gz-waves1::gz-waves1-hydrodynamics-system" for configuration "RelWithDebInfo"
set_property(TARGET gz-waves1::gz-waves1-hydrodynamics-system APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(gz-waves1::gz-waves1-hydrodynamics-system PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libgz-waves1-hydrodynamics-system.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libgz-waves1-hydrodynamics-system.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS gz-waves1::gz-waves1-hydrodynamics-system )
list(APPEND _IMPORT_CHECK_FILES_FOR_gz-waves1::gz-waves1-hydrodynamics-system "${_IMPORT_PREFIX}/lib/libgz-waves1-hydrodynamics-system.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
