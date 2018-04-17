cmake_minimum_required(VERSION 2.8)

PROJECT(NSGA-Parallel-Backend)
set (CMAKE_CXX_STANDARD 11)
SET(USE_VTK CACHE BOOL "Set to on to use vtk visualisation")
IF (USE_VTK)
	find_package(VTK REQUIRED)
		IF(VTK_FOUND)
    		include(${VTK_USE_FILE})
    		add_definitions(-DWITH_VTK)
		ENDIF()
ENDIF()

find_package(MPI REQUIRED)
include_directories(${MPI_CXX_INCLUDE_PATH})

set(CMAKE_AUTOMOC ON)
find_package(Qt5GUI)
find_package(Qt5Charts)
find_package(Qt5Widgets)

SET(USE_STATIC_BOOST CACHE BOOL "Set to on to use static boost libraries")
IF (USE_STATIC_BOOST)
    SET(Boost_USE_STATIC_LIBS ON)
    SET(Boost_USE_STATIC_RUNTIME ON)
ENDIF()

FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem system program_options mpi serialization timer chrono serialization date_time)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

add_definitions(-DVARIANT=4)

add_subdirectory(Checkpoints)
include_directories(Checkpoints)
add_subdirectory(Serialization)
include_directories(Serialization)

SET(BackendSources
	Crossover.hpp
	Individual.hpp
	Mutation.hpp
        Population.hpp
		Population.cpp
	Types.hpp
	)

SET(AllSources
    ${CheckpointsSources}
    ${SerializationSources}
    ${BackendSources}
    )

add_subdirectory(tests)





