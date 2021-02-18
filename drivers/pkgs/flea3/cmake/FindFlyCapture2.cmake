# FindFlyCapture2.cmake - Find FlyCapture2 SDK. Modified from FindEigen.cmake by
# alexs.mac@gmail.com  (Alex Stewart)
#
# This module defines the following variables:
#
# FlyCapture2_FOUND: TRUE if flycapture is found. FlyCapture2_INCLUDE_DIRS:
# Include directories for flycapture. FlyCapture2_LIBRARIES: Libraries for all
# flycapture component libraries and dependencies.
#
# Additionally, in Windows, it also defines the following variables
#
# FlyCapture2_LIBRARIES_DEBUG: Debug libraries for flycapture FlyCapture2_DLL:
# The DLL corresponding to the found library, for deployment purposes
# FlyCapture2_DLL_DEBUG : The DLL corresponding to the found debug library, for
# deployment purposes
#
# FlyCapture2_VERSION: Extracted from lib/libflycapture.so.x.y.z
# FlyCapture2_WORLD_VERSION: Equal to 2 if FlyCapture2_VERSION = 2.8.3
# FlyCapture2_MAJOR_VERSION: Equal to 8 if FlyCapture2_VERSION = 2.8.3
# FlyCapture2_MINOR_VERSION: Equal to 3 if FlyCapture2_VERSION = 2.8.3
#
# The following variables control the behaviour of this module:
#
# FlyCapture2_INCLUDE_DIR_HINTS: List of additional directories in which to
# search for flycapture includes, e.g: /foo/include.
# FlyCapture2_LIBRARY_DIR_HINTS: List of additional directories in which to
# search for flycapture libraries, e.g: /bar/lib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# FlyCapture2_INCLUDE_DIR: Include directory for flycapture, not including the
# include directory of any dependencies. FlyCapture2_LIBRARY: flycapture
# library, not including the libraries of any dependencies.

# Called if we failed to find flycapture or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports error
# message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(FlyCapture2_REPORT_NOT_FOUND REASON_MSG)
  unset(FlyCapture2_FOUND)
  unset(FlyCapture2_INCLUDE_DIRS)
  unset(FlyCapture2_LIBRARIES)
  unset(FlyCapture2_WORLD_VERSION)
  unset(FlyCapture2_MAJOR_VERSION)
  unset(FlyCapture2_MINOR_VERSION)
  # Make results of search visible in the CMake GUI if flycapture has not been
  # found so that user does not have to toggle to advanced view.
  mark_as_advanced(CLEAR FlyCapture2_INCLUDE_DIR)
  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
  # use the camelcase library name, not uppercase.
  if(FlyCapture2_FIND_QUIETLY)
    message(STATUS "Failed to find flycapture - " ${REASON_MSG} ${ARGN})
  elseif(FlyCapture2_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find flycapture - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use no priority which emits a message but
    # continues configuration and allows generation.
    message("-- Failed to find flycapture - " ${REASON_MSG} ${ARGN})
  endif()
endmacro(FlyCapture2_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs to
# system installs where both exist.
get_filename_component(FLYCAPTURE_DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)
list(APPEND FlyCapture2_CHECK_INCLUDE_DIRS ${FLYCAPTURE_DIR}/flycapture/include)
list(APPEND FlyCapture2_CHECK_LIBRARY_DIRS ${FLYCAPTURE_DIR}/flycapture/lib)
if(WIN32)
  set(FlyCapture2_HINTS
      "C:/Program Files/Point Grey Research/FlyCapture2"
      CACHE PATH "Path to FlyCapture2 SDK" FORCE)
endif()

# Check general hints
if(FlyCapture2_HINTS AND EXISTS ${FlyCapture2_HINTS})
  set(FlyCapture2_INCLUDE_DIR_HINTS ${FlyCapture2_HINTS}/include)
  set(FlyCapture2_LIBRARY_DIR_HINTS ${FlyCapture2_HINTS}/lib)
  if(WIN32)
    set(FlyCapture2_LIBRARY_DIR_HINTS ${FlyCapture2_HINTS}/lib64)
    set(FlyCapture2_DLL_DIR_HINTS ${FlyCapture2_HINTS}/bin64)
    if(MSVC)
      if(MSVC_VERSION GREATER_EQUAL "1800")
        if(MSVC_VERSION LESS "1900")
          set(FlyCapture2_LIBRARY_DIR_HINTS
              ${FlyCapture2_LIBRARY_DIR_HINTS}/vs2013)
          set(FlyCapture2_DLL_DIR_HINTS ${FlyCapture2_DLL_DIR_HINTS}/vs2013)
        else()
          set(FlyCapture2_LIBRARY_DIR_HINTS
              ${FlyCapture2_LIBRARY_DIR_HINTS}/vs2015)
          set(FlyCapture2_DLL_DIR_HINTS ${FlyCapture2_DLL_DIR_HINTS}/vs2015)
        endif()
      endif()
    else()
      message(
        WARNING
          "You don't seem to be using MSVC. FlyCapture is likely to not link!!!"
      )
      message(WARNING "Consider using the C library instead?")
      # This script would need further work to point you to that library
      set(FlyCapture2_LIBRARY_DIR_HINTS ${FlyCapture2_LIBRARY_DIR_HINTS}/vs2015)
      set(FlyCapture2_DLL_DIR_HINTS ${FlyCapture2_DLL_DIR_HINTS}/vs2015)
    endif()
  endif()

endif()

# Mark internally as found, then verify. FlyCapture2_REPORT_NOT_FOUND() unsets
# if called.
set(FlyCapture2_FOUND TRUE)

# Search supplied hint directories first if supplied. Find include directory for
# flycapture
if(WIN32)
  # unfortunately Flycapture installs itself in an un-prepended include path in
  # Windows
  find_path(
    FlyCapture2_INCLUDE_DIR
    NAMES FlyCapture2.h
    PATHS ${FlyCapture2_INCLUDE_DIR_HINTS} ${FlyCapture2_CHECK_INCLUDE_DIRS})
else()
  find_path(
    FlyCapture2_INCLUDE_DIR
    NAMES flycapture/FlyCapture2.h
    PATHS ${FlyCapture2_INCLUDE_DIR_HINTS} ${FlyCapture2_CHECK_INCLUDE_DIRS})
endif()
if(NOT FlyCapture2_INCLUDE_DIR OR NOT EXISTS ${FlyCapture2_INCLUDE_DIR})
  flycapture2_report_not_found(
    "Could not find flycapture include directory, set FlyCapture2_INCLUDE_DIR to "
    "path to flycapture include directory,"
    "e.g. /opt/flycapture.")
else()
  message(STATUS "flycapture include dir found: " ${FlyCapture2_INCLUDE_DIR})
endif()

if(NOT WIN32)
  # Find library directory for flycapture
  find_library(
    FlyCapture2_LIBRARY
    NAMES libflycapture.so
    PATHS ${FlyCapture2_LIBRARY_DIR_HINTS} ${FlyCapture2_CHECK_LIBRARY_DIRS})
else()

  if(MSVC)
    if(MSVC_VERSION LESS "1400")
      flycapture2_report_not_found(
        "FlyCapture2 SDK not available for VC earlier than VC9"
        "Please use an SDK newer than Visual Studio 2005")
    elseif(MSVC_VERSION LESS "1500")
      set(vTail "")
    elseif(MSVC_VERSION LESS "1600")
      set(vTail "_v90")
    elseif(MSVC_VERSION LESS "1700")
      set(vTail "_v100")
    elseif(MSVC_VERSION LESS "1800")
      set(vTail "_v110")
    elseif(MSVC_VERSION LESS "1900")
      set(vTail "_v120")
    elseif(MSVC_VERSION LESS_EQUAL "1910")
      if(MSVC_VERSION EQUAL "1910")
        message(WARNING "I'm assuming from version numbers that VC15 will link"
                        "But this is nothing but a guess")
      endif()

      set(vTail "_v140")
    endif()
  else()
    # Have the latest one and hope for the best
    set(vTail "_v140")
  endif()

  # message("FlyCapture2_LIBRARY_DIR_HINTS: ${FlyCapture2_LIBRARY_DIR_HINTS} ")
  # message("FlyCapture2_LIB_NAME: FlyCapture2${vTail}.lib")
  # message("FlyCapture2_DLL_NAME: FlyCapture2${vTail}.dll")
  # message("FlyCapture2_LIB_NAME_DEBUG: FlyCapture2d${vTail}.lib")
  # message("FlyCapture2_DLL_NAME_DEBUG: FlyCapture2d${vTail}.dll")

  find_library(
    FlyCapture2_LIBRARY
    NAMES FlyCapture2${vTail}.lib
    PATHS ${FlyCapture2_LIBRARY_DIR_HINTS} ${FlyCapture2_CHECK_LIBRARY_DIRS})
  find_library(
    FlyCapture2_LIBRARY_DEBUG
    NAMES FlyCapture2d${vTail}.lib
    PATHS ${FlyCapture2_LIBRARY_DIR_HINTS} ${FlyCapture2_CHECK_LIBRARY_DIRS})
  find_file(
    FlyCapture2_RUNTIME
    NAMES FlyCapture2${vTail}.dll
    PATHS ${FlyCapture2_DLL_DIR_HINTS} ${FlyCapture2_CHECK_LIBRARY_DIRS})
  find_file(
    FlyCapture2_RUNTIME_DEBUG
    NAMES FlyCapture2d${vTail}.dll
    PATHS ${FlyCapture2_DLL_DIR_HINTS} ${FlyCapture2_CHECK_LIBRARY_DIRS})

endif()

if(NOT FlyCapture2_LIBRARY OR NOT EXISTS ${FlyCapture2_LIBRARY})
  flycapture2_report_not_found(
    "Could not find flycapture library, set FlyCapture2_LIBRARY "
    "to full path to flycapture library directory.")
elseif(MSVC AND MSVC_VERSION LESS "1400")
  flycapture2_report_not_found(
    "FlyCapture2 SDK not available for VC earlier than VC9"
    "Please use an SDK newer than Visual Studio 2005")
else()

  get_filename_component(FlyCapture2_LIBRARY_DIR ${FlyCapture2_LIBRARY}
                         DIRECTORY)
  message(STATUS "flycapture library dir found: " ${FlyCapture2_LIBRARY_DIR})

endif()

# Extract flycapture version
if(FlyCapture2_LIBRARY_DIR)
  if(NOT WIN32)
    file(
      GLOB FlyCapture2_LIBS
      RELATIVE ${FlyCapture2_LIBRARY_DIR}
      ${FlyCapture2_LIBRARY_DIR}/libflycapture.so.[0-9].[0-9].[0-9])
  endif()
endif()

# Catch case when caller has set FlyCapture2_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are invalid,
# otherwise we would report the library as found.
if(WIN32)
  if(FlyCapture2_INCLUDE_DIR AND NOT EXISTS
                                 ${FlyCapture2_INCLUDE_DIR}/FlyCapture2.h)
    flycapture2_report_not_found(
      "Caller defined FlyCapture2_INCLUDE_DIR: " ${FlyCapture2_INCLUDE_DIR}
      " does not contain FlyCapture2.h header.")
  endif()
else()
  if(FlyCapture2_INCLUDE_DIR
     AND NOT EXISTS ${FlyCapture2_INCLUDE_DIR}/flycapture/FlyCapture2.h)
    flycapture2_report_not_found(
      "Caller defined FlyCapture2_INCLUDE_DIR: " ${FlyCapture2_INCLUDE_DIR}
      " does not contain flycapture/FlyCapture2.h header.")
  endif()
endif()

# Set standard CMake FindPackage variables if found.
if(FlyCapture2_FOUND)
  set(FlyCapture2_INCLUDE_DIRS ${FlyCapture2_INCLUDE_DIR})
  if(WIN32)
    get_filename_component(FlyCapture2_LIBRARIES ${FlyCapture2_LIBRARY} NAME)
    get_filename_component(FlyCapture2_LIBRARIES_DEBUG
                           ${FlyCapture2_LIBRARY_DEBUG} NAME)
    # The use for these is "install(FILES)" so full paths are fine
    set(FlyCapture2_DLL ${FlyCapture2_RUNTIME})
    set(FlyCapture2_DLL_DEBUG ${FlyCapture2_RUNTIME_DEBUG})
  else()
    file(GLOB FlyCapture2_LIBRARIES ${FlyCapture2_LIBRARY})
  endif()
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
if(FlyCapture2_FOUND)
  find_package_handle_standard_args(
    FlyCapture2 DEFAULT_MSG FlyCapture2_INCLUDE_DIRS FlyCapture2_LIBRARIES)
endif()

# Only mark internal variables as advanced if we found flycapture, otherwise
# leave it visible in the standard GUI for the user to set manually.
if(FlyCapture2_FOUND)
  mark_as_advanced(FORCE FlyCapture2_INCLUDE_DIR FlyCapture2_LIBRARY)
endif()
