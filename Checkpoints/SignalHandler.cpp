//
// Created by a1091793 on 11/4/17.
//

#include "SignalHandler.hpp"

std::map< int, bool > signals_called;

void signal_handler(int sig)
{
    signals_called[sig] = true;
}