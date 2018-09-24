cmake_minimum_required(VERSION 2.8)
project(ibeosdk_tools)

# parse tools files
#######################################################

file(GLOB TOOL_SOURCES *.cpp)

# boost
#######################################################
include_directories(${CMAKE_SOURCE_DIR}/src)

find_package(Boost REQUIRED COMPONENTS system thread program_options date_time)
include_directories(${Boost_INCLUDE_DIRS})

# create demos
#######################################################
foreach(tool ${TOOL_SOURCES})

	if(${MSVC})
		source_group("sources" FILES ${tool})
	endif()

	get_filename_component(toolName "${tool}" NAME_WE)

	add_executable(${toolName} ${tool})
	target_link_libraries(${toolName} ibeosdk)

	if(${MSVC})
		target_compile_options(${toolName} PUBLIC /MP)
		target_compile_definitions(${toolName} PUBLIC _WIN32 _DEBUG _WINDOWS _WIN32_WINNT=0x502 BOOST_ALL_NO_LIB WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS _CONSOLE)
	endif()

	# copy to bin folder
	add_custom_command(TARGET ${toolName} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${toolName}> ${IBEOSDK_BINDIR})
endforeach()

#######################################################
