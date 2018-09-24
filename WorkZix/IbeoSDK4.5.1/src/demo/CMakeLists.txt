cmake_minimum_required(VERSION 2.8)
project(ibeosdk_demos)

# parse demo files
#######################################################

file(GLOB DEMO_SOURCES *.cpp)

# boost
#######################################################
include_directories(${CMAKE_SOURCE_DIR}/src)

find_package(Boost REQUIRED COMPONENTS system thread program_options date_time)
include_directories(${Boost_INCLUDE_DIRS})

# create demos
#######################################################
foreach(demo ${DEMO_SOURCES})

	if(${MSVC})
		source_group("sources" FILES ${demo})
	endif()

	get_filename_component(demoName "${demo}" NAME_WE)
	if ( (${MSVC} STREQUAL "") OR (NOT ${demoName} STREQUAL "IbeoSdkDataLoggerDemo") )
		add_executable(${demoName} ${demo})
		target_link_libraries(${demoName} ibeosdk)

		if(${MSVC})
			target_compile_options(${demoName} PUBLIC /MP)
			target_compile_definitions(${demoName} PUBLIC _WIN32 _DEBUG _WINDOWS _WIN32_WINNT=0x502 BOOST_ALL_NO_LIB WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS _CONSOLE)
		endif()

		# copy to bin folder
		add_custom_command(TARGET ${demoName} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${demoName}> ${IBEOSDK_BINDIR})
	endif()
endforeach()

#######################################################
