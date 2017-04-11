//
// Created by a1091793 on 11/4/17.
// INspired by the eo implementation (as I had no experience in handling signals at this point.
//

#ifndef GEON_ZONAL_OPT_SIGNALCHECKPOINT_H
#define GEON_ZONAL_OPT_SIGNALCHECKPOINT_H

#include "../Checkpoint.hpp"
#include <csignal>
#include <map>
#include "SignalHandler.hpp"



class SignalCheckpoint : public CheckpointBase
{
private:
    int signal_being_handled;

public:
    SignalCheckpoint(int _signal_to_handle = SIGINT)
            : signal_being_handled(_signal_to_handle)
    {
        signal(signal_being_handled, signal_handler);
    }

    bool
    operator()(PopulationSPtr population)
    {
        bool& has_been_called = signals_called[signal_being_handled];
        if (has_been_called)
        {
            std::cout << "Terminating. Received signal: " << (signal_being_handled) << std::endl;
            return (false);
        }
        return (true);
    }
};


#endif //GEON_ZONAL_OPT_SIGNALCHECKPOINT_H
