SET(BackendSourcesT
        ../Checkpoint.hpp

        ../Comparator.hpp
        ../Crossover.hpp
        ../DebsCrowdingDistance.hpp
        ../DebsNondominatedSorting.hpp
        ../Evaluation.hpp
        ../Individual.hpp
        ../Merge.hpp
        ../Mutation.hpp
        ../NSGAII.hpp
        ../ParallelEvaluator.hpp
        ../Population.hpp
        ../ProblemDefinitions.hpp
        ../Selection.hpp
        ../Types.hpp
        )

SET(CheckpointsSourcesT
        ../Checkpoints/MaxGenCheckpoint.hpp
        ../Checkpoints/SavePopCheckpoint.hpp
        ../Checkpoints/SerialiseCheckpoint.hpp
        ../Checkpoints/PlotFronts.hpp
        ../Checkpoints/colours.hpp
        ../Checkpoints/ResetMutationXoverFlags.hpp
        ../Checkpoints/MailCheckpoint.hpp
        ../Checkpoints/MetricLinePlot.hpp
        )

SET(SerializationSourcesT
        ../Serialization/SerializeBoostPath.hpp
        ../Serialization/SerializeRng.hpp
        )

SET(MetricSourcesT
        ../Metrics/hv.c
        ../Metrics/hv.h
        ../Metrics/Hypervolume.hpp
        )

SET(SampleProblemsT
        ../SampleProblems/TestFunctions.hpp
        )

SET(AllSourcesT
        ${CheckpointsSourcesT}
        ${SerializationSourcesT}
        ${BackendSourcesT}
        ${MetricSourcesT}
        ${SampleProblemsT}
        )

SET( DebsSBXTesterSources
        DebsSBXCrossoverTester.cpp
    )
	
ADD_EXECUTABLE(DebsSBXTester ${DebsSBXTesterSources} ${AllSourcesT})

IF(USE_VTK)
    if(VTK_LIBRARIES)
        target_link_libraries(DebsSBXTester ${VTK_LIBRARIES} ${MPI_LIBRARIES})
    else()
        target_link_libraries(DebsSBXTester vtkHybrid vtkWidgets)
    endif()
ENDIF()

	
set_property(TARGET DebsSBXTester PROPERTY CXX_STANDARD 11)





SET (DebsPolynomialMutationTesterSources
    DebsPolynomialMutationTester.cpp
)

ADD_EXECUTABLE(DevsPolynomialMutationTester ${DebsPolynomialMutationTesterSources} ${AllSourcesT})

IF(USE_VTK)
    if(VTK_LIBRARIES)
	  target_link_libraries(DevsPolynomialMutationTester ${VTK_LIBRARIES} ${MPI_LIBRARIES})
	else()
	  target_link_libraries(DevsPolynomialMutationTester vtkHybrid vtkWidgets)
	endif()
ENDIF()
	
set_property(TARGET DevsPolynomialMutationTester PROPERTY CXX_STANDARD 11)




SET(FonTestProblemSources 
	TestNSGAII_FON.cpp
)
	
ADD_EXECUTABLE(FonTEstProblem ${FonTestProblemSources}  ${AllSourcesT})
target_link_libraries(FonTEstProblem ${Boost_LIBRARIES})

IF(USE_VTK)
    if(VTK_LIBRARIES)
        target_link_libraries(FonTEstProblem ${VTK_LIBRARIES} ${MPI_LIBRARIES})
    else()
        target_link_libraries(FonTEstProblem vtkHybrid vtkWidgets)
    endif()
ENDIF()

set_property(TARGET FonTEstProblem PROPERTY CXX_STANDARD 11)



SET(FonTestSources
    TestFON.cpp
)

ADD_EXECUTABLE(TestFON ${FonTestSources}  ${AllSourcesT})

set_property(TARGET TestFON PROPERTY CXX_STANDARD 11)


SET(DebsCrowdingCalcTestSources
    TestDebsCrowdingDistance.cpp
)

        ADD_EXECUTABLE(TestDebsCrowding ${DebsCrowdingCalcTestSources} )

IF(USE_VTK)
	if(VTK_LIBRARIES)
		target_link_libraries(TestDebsCrowding ${VTK_LIBRARIES} ${MPI_LIBRARIES})
	else()
		target_link_libraries(TestDebsCrowding vtkHybrid vtkWidgets)
	endif()
ENDIF()

set_property(TARGET TestDebsCrowding PROPERTY CXX_STANDARD 11)


SET (TestParallelEvalSources
        TestParallelEvaluator.cpp
)

ADD_EXECUTABLE(TestParallelEval ${TestParallelEvalSources}  ${AllSourcesT})
target_link_libraries(TestParallelEval ${MPI_LIBRARIES} ${Boost_LIBRARIES})

if(MPI_COMPILE_FLAGS)
set_target_properties(TestParallelEval PROPERTIES
COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
endif()

if(MPI_LINK_FLAGS)
set_target_properties(TestParallelEval PROPERTIES
LINK_FLAGS "${MPI_LINK_FLAGS}")
endif()

set_property(TARGET TestParallelEval PROPERTY CXX_STANDARD 11)


SET(TestFONParallelSources
        TestFonParallel.cpp
)

ADD_EXECUTABLE(TestParallelFON ${TestFONParallelSources}  ${AllSourcesT})

IF(USE_VTK)
    if(VTK_LIBRARIES)
        target_link_libraries(TestParallelFON ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    else()
        target_link_libraries(TestParallelFON vtkHybrid vtkWidgets ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    endif()
ELSE()
    target_link_libraries(TestParallelFON ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()


if(MPI_COMPILE_FLAGS)
  set_target_properties(TestParallelFON PROPERTIES
    COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
endif()

if(MPI_LINK_FLAGS)
  set_target_properties(TestParallelFON PROPERTIES
    LINK_FLAGS "${MPI_LINK_FLAGS}")
endif()

set_property(TARGET TestParallelFON PROPERTY CXX_STANDARD 11)


SET(TestParallelSpeedup
        TestParallelSpeedup.cpp
)

ADD_EXECUTABLE(TestParallelSpeedup ${TestParallelSpeedup}  ${AllSourcesT} ${AllSourcesT})
IF(USE_VTK)
    if(VTK_LIBRARIES)
        target_link_libraries(TestParallelSpeedup ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    else()
        target_link_libraries(TestParallelSpeedup vtkHybrid vtkWidgets ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    endif()
ELSE()
    target_link_libraries(TestParallelSpeedup ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()


if(MPI_COMPILE_FLAGS)
  set_target_properties(TestParallelSpeedup PROPERTIES
    COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
endif()

if(MPI_LINK_FLAGS)
  set_target_properties(TestParallelSpeedup PROPERTIES
    LINK_FLAGS "${MPI_LINK_FLAGS}")
endif()

set_property(TARGET TestParallelSpeedup PROPERTY CXX_STANDARD 11)


SET(TestIndiSerialisatn
        testserialisation.cpp
   )
ADD_EXECUTABLE(TestIndSerltn ${TestIndiSerialisatn})
target_link_libraries(TestIndSerltn ${Boost_LIBRARIES})
set_property(TARGET TestIndSerltn PROPERTY CXX_STANDARD 11)
