# Locate an official MuJoCo release or the native library bundled in a Python wheel.

set(MUJOCO_ROOT "" CACHE PATH
  "MuJoCo root (official release directory or Python package directory)")

set(_mujoco_roots)
if(MUJOCO_ROOT)
  list(APPEND _mujoco_roots "${MUJOCO_ROOT}")
elseif(DEFINED ENV{MUJOCO_ROOT})
  list(APPEND _mujoco_roots "$ENV{MUJOCO_ROOT}")
endif()

if(DEFINED ENV{CONDA_PREFIX})
  file(GLOB _mujoco_conda_roots
    "$ENV{CONDA_PREFIX}/lib/python*/site-packages/mujoco"
    "$ENV{CONDA_PREFIX}/Lib/site-packages/mujoco")
  list(APPEND _mujoco_roots ${_mujoco_conda_roots})
endif()

set(_mujoco_include_hints)
set(_mujoco_library_hints)
set(_mujoco_runtime_hints)
foreach(_root IN LISTS _mujoco_roots)
  list(APPEND _mujoco_include_hints "${_root}/include")
  list(APPEND _mujoco_library_hints "${_root}/lib" "${_root}/bin" "${_root}")
  list(APPEND _mujoco_runtime_hints "${_root}/bin" "${_root}/lib" "${_root}")
endforeach()

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${_mujoco_include_hints})

if(WIN32)
  find_library(MUJOCO_LIBRARY
    NAMES mujoco
    HINTS ${_mujoco_library_hints})
  find_file(MUJOCO_RUNTIME
    NAMES mujoco.dll
    HINTS ${_mujoco_runtime_hints})
else()
  find_library(MUJOCO_LIBRARY
    NAMES mujoco
    HINTS ${_mujoco_library_hints})

  if(NOT MUJOCO_LIBRARY)
    set(_mujoco_versioned_libraries)
    foreach(_dir IN LISTS _mujoco_library_hints)
      if(APPLE)
        file(GLOB _matches "${_dir}/libmujoco*.dylib")
      else()
        file(GLOB _matches "${_dir}/libmujoco.so.*")
      endif()
      list(APPEND _mujoco_versioned_libraries ${_matches})
    endforeach()
    list(SORT _mujoco_versioned_libraries COMPARE NATURAL ORDER DESCENDING)
    if(_mujoco_versioned_libraries)
      list(GET _mujoco_versioned_libraries 0 MUJOCO_LIBRARY)
    endif()
  endif()

  set(MUJOCO_RUNTIME "${MUJOCO_LIBRARY}")
endif()

include(FindPackageHandleStandardArgs)
if(WIN32)
  find_package_handle_standard_args(MuJoCo
    REQUIRED_VARS MUJOCO_INCLUDE_DIR MUJOCO_LIBRARY MUJOCO_RUNTIME)
else()
  find_package_handle_standard_args(MuJoCo
    REQUIRED_VARS MUJOCO_INCLUDE_DIR MUJOCO_LIBRARY)
endif()

if(MuJoCo_FOUND AND NOT TARGET mujoco::mujoco)
  if(WIN32)
    add_library(mujoco::mujoco SHARED IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_IMPLIB "${MUJOCO_LIBRARY}"
      IMPORTED_LOCATION "${MUJOCO_RUNTIME}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  else()
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  endif()

  get_filename_component(MUJOCO_RUNTIME_DIR "${MUJOCO_RUNTIME}" DIRECTORY)
endif()

mark_as_advanced(MUJOCO_INCLUDE_DIR MUJOCO_LIBRARY MUJOCO_RUNTIME)
