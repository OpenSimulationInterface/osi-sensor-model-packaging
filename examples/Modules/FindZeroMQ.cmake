# Try to find ZeroMQ Library

find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)

find_path(ZeroMQ_INCLUDE_DIR
	NAMES zmq.hpp
	PATHS ${PC_ZeroMQ_INCLUDE_DIRS})

find_library(ZeroMQ_LIBRARY
	NAMES libzmq${CMAKE_SHARED_LIBRARY_SUFFIX}
	PATHS ${PC_ZeroMQ_LIBRARY_DIRS})

find_library(ZeroMQ_LIBRARY_STATIC
	NAMES libzmq${CMAKE_STATIC_LIBRARY_SUFFIX}
	PATHS ${PC_ZeroMQ_LIBRARY_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZeroMQ DEFAULT_MSG ZeroMQ_LIBRARY ZeroMQ_LIBRARY_STATIC ZeroMQ_INCLUDE_DIR)
mark_as_advanced(ZeroMQ_INCLUDE_DIR ZeroMQ_LIBRARY ZeroMQ_LIBRARY_STATIC)
