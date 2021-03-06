
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")

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
        ../Checkpoints/SaveFirstFrontCheckpoint.hpp
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
        target_link_libraries(DebsSBXTester ${VTK_LIBRARIES} ${Boost_LIBRARIES})
    else()
        target_link_libraries(DebsSBXTester vtkHybrid vtkWidgets ${Boost_LIBRARIES})
    endif()
ELSE()
        target_link_libraries(DebsSBXTester ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()

	
set_property(TARGET DebsSBXTester PROPERTY CXX_STANDARD 11)





SET (DebsPolynomialMutationTesterSources
    DebsPolynomialMutationTester.cpp
)

ADD_EXECUTABLE(DevsPolynomialMutationTester ${DebsPolynomialMutationTesterSources} ${AllSourcesT})

IF(USE_VTK)
    if(VTK_LIBRARIES)
          target_link_libraries(DevsPolynomialMutationTester ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
	else()
          target_link_libraries(DevsPolynomialMutationTester vtkHybrid vtkWidgets ${Boost_LIBRARIES})
        endif()
ELSE()
        target_link_libraries(DevsPolynomialMutationTester ${MPI_LIBRARIES} ${Boost_LIBRARIES} Qt5::Widgets Qt5::Gui Qt5::Charts)
ENDIF()
	
set_property(TARGET DevsPolynomialMutationTester PROPERTY CXX_STANDARD 11)




SET(FonTestProblemSources 
	TestNSGAII_FON.cpp
)
	
ADD_EXECUTABLE(FonTEstProblem ${FonTestProblemSources}  ${AllSourcesT})
target_link_libraries(FonTEstProblem ${Boost_LIBRARIES})

IF(USE_VTK)
    if(VTK_LIBRARIES)
        target_link_libraries(FonTEstProblem ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    else()
        target_link_libraries(FonTEstProblem vtkHybrid vtkWidgets ${Boost_LIBRARIES})
    endif()
ELSE()
            target_link_libraries(FonTEstProblem ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()

set_property(TARGET FonTEstProblem PROPERTY CXX_STANDARD 11)



SET(FonTestSources
    TestFON.cpp
)

ADD_EXECUTABLE(TestFON ${FonTestSources}  ${AllSourcesT})

set_property(TARGET TestFON PROPERTY CXX_STANDARD 11)

IF(USE_VTK)
    if(VTK_LIBRARIES)
        target_link_libraries(TestFON ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    else()
        target_link_libraries(TestFON vtkHybrid vtkWidgets ${MPI_LIBRARIES} ${Boost_LIBRARIES})
    endif()
ELSE()
    target_link_libraries(TestFON ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()


SET(DebsCrowdingCalcTestSources
    TestDebsCrowdingDistance.cpp
)

        ADD_EXECUTABLE(TestDebsCrowding ${DebsCrowdingCalcTestSources} )

IF(USE_VTK)
	if(VTK_LIBRARIES)
                target_link_libraries(TestDebsCrowding ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
	else()
                target_link_libraries(TestDebsCrowding vtkHybrid vtkWidgets ${Boost_LIBRARIES})
        endif()
ELSE()
                    target_link_libraries(TestDebsCrowding ${MPI_LIBRARIES} ${Boost_LIBRARIES})
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


#if(MPI_COMPILE_FLAGS)
#  set_target_properties(TestParallelFON PROPERTIES
#    COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
#endif()

#if(MPI_LINK_FLAGS)
#  set_target_properties(TestParallelFON PROPERTIES
#    LINK_FLAGS "${MPI_LINK_FLAGS}")
#endif()

#set_property(TARGET TestParallelFON PROPERTY CXX_STANDARD 11)


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


#if(MPI_COMPILE_FLAGS)
#  set_target_properties(TestParallelSpeedup PROPERTIES
#    COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
#endif()

#if(MPI_LINK_FLAGS)
#  set_target_properties(TestParallelSpeedup PROPERTIES
#    LINK_FLAGS "${MPI_LINK_FLAGS}")
#endif()

#set_property(TARGET TestParallelSpeedup PROPERTY CXX_STANDARD 11)


SET(TestIndiSerialisatn
        testserialisation.cpp
   )
ADD_EXECUTABLE(TestIndSerltn ${TestIndiSerialisatn})
target_link_libraries(TestIndSerltn ${Boost_LIBRARIES})
set_property(TARGET TestIndSerltn PROPERTY CXX_STANDARD 11)



SET(TestIntMinSerialSources
TestIntMinSerial.cpp
)

ADD_EXECUTABLE(TestSerialIntMin ${TestIntMinSerialSources}  ${AllSourcesT})

IF(USE_VTK)
if(VTK_LIBRARIES)
target_link_libraries(TestSerialIntMin ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
else()
target_link_libraries(TestSerialIntMin vtkHybrid vtkWidgets ${MPI_LIBRARIES} ${Boost_LIBRARIES})
endif()
ELSE()
target_link_libraries(TestSerialIntMin ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()


SET(TestIntMinParallelSources
TestIntMinParallel.cpp
)

ADD_EXECUTABLE(TestParallelIntMin ${TestIntMinParallelSources}  ${AllSourcesT})

IF(USE_VTK)
if(VTK_LIBRARIES)
target_link_libraries(TestParallelIntMin ${VTK_LIBRARIES} ${MPI_LIBRARIES} ${Boost_LIBRARIES})
else()
target_link_libraries(TestParallelIntMin vtkHybrid vtkWidgets ${MPI_LIBRARIES} ${Boost_LIBRARIES})
endif()
ELSE()
target_link_libraries(TestParallelIntMin ${MPI_LIBRARIES} ${Boost_LIBRARIES})
ENDIF()

SET(TestIntUniformMutationSources
        TestUniformIntMutation.cpp)

# Instruct CMake to run moc automatically when needed.
ADD_EXECUTABLE(TestUniformIntMutation ${TestIntUniformMutationSources} ${AllSourcesT})
target_link_libraries(TestUniformIntMutation Qt5::Widgets Qt5::Gui Qt5::Charts  ${Boost_LIBRARIES})

SET(TestIntCrossoverSources
        TestIntCrossover.cpp)

# Instruct CMake to run moc automatically when needed.
ADD_EXECUTABLE(TestIntCrossover ${TestIntCrossoverSources} ${AllSourcesT})
target_link_libraries(TestIntCrossover Qt5::Widgets Qt5::Gui Qt5::Charts  ${Boost_LIBRARIES})

SET(TestIntCrossAndMutSources
        TestIntMutAndCross.cpp)

# Instruct CMake to run moc automatically when needed.
ADD_EXECUTABLE(TestIntMutAndCross ${TestIntCrossAndMutSources} ${AllSourcesT})
target_link_libraries(TestIntMutAndCross Qt5::Widgets Qt5::Gui Qt5::Charts  ${Boost_LIBRARIES})
