cmake_minimum_required(VERSION 2.8)

PROJECT(NSGA-Parallel-Backend)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(MPI REQUIRED)
include_directories(${MPI_CXX_INCLUDE_PATH})

SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_USE_STATIC_RUNTIME ON)
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem system program_options mpi serialization timer chrono)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

SET(BackendSources
	Crossover.hpp
	Individual.hpp
	Mutation.hpp
	parallel_nsgaII_backend.cpp
	parallel_nsgaII_backend.hpp
	parallel_nsgaII_backendPriv.hpp
	Population.hpp
	Types.hpp
	)

add_subdirectory(tests)




