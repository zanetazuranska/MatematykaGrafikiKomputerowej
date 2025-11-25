
cmake_minimum_required(VERSION 3.15)

project(Zdefinowanie_przestrzeni LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Collect all C/C++ source files recursively. CONFIGURE_DEPENDS makes CMake
# re-run if files are added/removed while using a multi-config generator or
# during incremental builds. This requires CMake >= 3.12 (we require 3.15).
# Collect all C/C++ source files recursively from the source tree. Use a
# temporary list `ALL_SRC` then filter out any files that live under the
# build directory (`${CMAKE_BINARY_DIR}`) to avoid accidentally picking up
# generated sources (e.g. CMake compiler-id sources) when an in-tree build
# or a `build/` subdir exists inside the source tree.
file(GLOB_RECURSE ALL_SRC CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

set(SRC_FILES)
foreach(_f IN LISTS ALL_SRC)
	# Exclude files that are inside the build tree
	string(FIND "${_f}" "${CMAKE_BINARY_DIR}" _in_build)
	if(_in_build EQUAL -1)
		list(APPEND SRC_FILES "${_f}")
	endif()
endforeach()

if(NOT SRC_FILES)
	message(FATAL_ERROR "No source files found in ${CMAKE_CURRENT_SOURCE_DIR} (after filtering out build tree)")
endif()

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-parameter)
	endif()
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
	ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
	LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Create IDE-friendly source groups (mirrors folder tree)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "Source Files" FILES ${SRC_FILES})

install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION bin
)