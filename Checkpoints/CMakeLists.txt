SET(CheckpointsSources
        MaxGenCheckpoint.hpp
        PlotFronts.hpp
        ResetMutationXoverFlags.hpp
        SavePopCheckpoint.hpp
        SerialiseCheckpoint.hpp
        SignalCheckpoint.hpp
        SignalHandler.cpp
        SignalHandler.hpp
    )

source_group(Checkpoints FILES ${CheckpointsSources})
