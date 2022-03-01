cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 17)

project(TelloPoseControl)
find_package(Threads REQUIRED)
# setup CUDA
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/utils/cuda" )
find_package(CUDA)
message("-- CUDA version: ${CUDA_VERSION}")

set(
	CUDA_NVCC_FLAGS
	${CUDA_NVCC_FLAGS};
	-O3
	-gencode arch=compute_53,code=sm_53
	-gencode arch=compute_62,code=sm_62
)

if(CUDA_VERSION_MAJOR GREATER 9)
	message("-- CUDA ${CUDA_VERSION_MAJOR} detected, enabling SM_72")

	set(
		CUDA_NVCC_FLAGS
		${CUDA_NVCC_FLAGS};
		-gencode arch=compute_72,code=sm_72
	)
endif()

set(SOURCE_FILES
GamepadControl.cpp
DronControl.cpp
)

set(HEADER_FILES
GamepadControl.h
DronControl.h
)

include_directories(
	${CMAKE_SOURCE_DIR}/include/jetson-utils
	${CMAKE_SOURCE_DIR}/include/jetson-inference
	${CUDA_INCLUDE_DIRS}
)

link_directories(
${CMAKE_SOURCE_DIR}/lib
)

link_directories(${PROJECT_NAME} /usr/local/lib)
add_executable(${PROJECT_NAME} "main.cpp"
	${SOURCE_FILES}
	${HEADER_FILES})

target_link_libraries(${CMAKE_PROJECT_NAME}
	libctello.so
	Threads::Threads
	${CUDA_LIBRARIES}
	libjetson-utils.so
	libjetson-inference.so)
