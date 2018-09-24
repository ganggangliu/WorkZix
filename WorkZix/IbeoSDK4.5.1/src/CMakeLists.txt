cmake_minimum_required(VERSION 2.8)

add_subdirectory(ibeosdk)

IF (WITH_DEMOS)
	add_subdirectory(demo)
ENDIF()

IF (WITH_TOOLS)
	add_subdirectory(tools)
ENDIF()

IF (WITH_UNITTESTS)
	add_subdirectory(unittests)
ENDIF()
