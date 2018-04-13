cmake_minimum_required(VERSION 3.5)
project(OSMPCNetworkProxy)

set(PUBLIC_LOGGING OFF CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING OFF CACHE BOOL "Enable private logging to file")
if(WIN32)
	set(PRIVATE_LOG_PATH_CPROXY "C:/TEMP/OSMPCNetworkProxyLog.log" CACHE FILEPATH "Path to write private log file to")
else()
	set(PRIVATE_LOG_PATH_CPROXY "/tmp/OSMPCNetworkProxyLog.log" CACHE FILEPATH "Path to write private log file to")
endif()
set(VERBOSE_FMI_LOGGING OFF CACHE BOOL "Enable detailed FMI function logging")
set(DEBUG_BREAKS OFF CACHE BOOL "Enable debugger traps for debug builds of FMU")
set(FMU_DEFAULT_ADDRESS "127.0.0.1" CACHE STRING "Default address for connections")
set(FMU_DEFAULT_PORT "3456" CACHE STRING "Default port for connections")
set(FMU_LISTEN OFF CACHE BOOL "Create FMU that passively listens (server mode)")

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)
configure_file(modelDescription.in.xml modelDescription.xml @ONLY)

add_library(OSMPCNetworkProxy SHARED OSMPCNetworkProxy.c)
set_target_properties(OSMPCNetworkProxy PROPERTIES PREFIX "")
target_compile_definitions(OSMPCNetworkProxy PRIVATE "FMU_SHARED_OBJECT")
target_compile_definitions(OSMPCNetworkProxy PRIVATE "FMU_GUID=\"${FMUGUID}\"")
target_compile_definitions(OSMPCNetworkProxy PRIVATE "FMU_DEFAULT_ADDRESS=\"${FMU_DEFAULT_ADDRESS}\"")
target_compile_definitions(OSMPCNetworkProxy PRIVATE "FMU_DEFAULT_PORT=\"${FMU_DEFAULT_PORT}\"")
if(PRIVATE_LOGGING)
	file(TO_NATIVE_PATH ${PRIVATE_LOG_PATH_CPROXY} PRIVATE_LOG_PATH_CPROXY_NATIVE)
	string(REPLACE "\\" "\\\\" PRIVATE_LOG_PATH_CPROXY_ESCAPED ${PRIVATE_LOG_PATH_CPROXY_NATIVE})
	target_compile_definitions(OSMPCNetworkProxy PRIVATE
		"PRIVATE_LOG_PATH=\"${PRIVATE_LOG_PATH_CPROXY_ESCAPED}\"")
endif()
target_compile_definitions(OSMPCNetworkProxy PRIVATE
    $<$<BOOL:${FMU_LISTEN}>:FMU_LISTEN>
	$<$<BOOL:${PUBLIC_LOGGING}>:PUBLIC_LOGGING>
	$<$<BOOL:${VERBOSE_FMI_LOGGING}>:VERBOSE_FMI_LOGGING>
	$<$<BOOL:${DEBUG_BREAKS}>:DEBUG_BREAKS>)
if(WIN32)
	target_link_libraries(OSMPCNetworkProxy wsock32 ws2_32)
endif()

if(WIN32)
	if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
		set(FMI_BINARIES_PLATFORM "win64")
	else()
		set(FMI_BINARIES_PLATFORM "win32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
		set(FMI_BINARIES_PLATFORM "linux64")
	else()
		set(FMI_BINARIES_PLATFORM "linux32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
		set(FMI_BINARIES_PLATFORM "darwin64")
	else()
		set(FMI_BINARIES_PLATFORM "darwin32")
	endif()
endif()

add_custom_command(TARGET OSMPCNetworkProxy
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/sources"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/modelDescription.xml"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/OSMPCNetworkProxy.c" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/OSMPCNetworkProxy.c"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/OSMPCNetworkProxy.h" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/OSMPCNetworkProxy.h"
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OSMPCNetworkProxy> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "../OSMPCNetworkProxy.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")
