//
// Created by a1091793 on 11/4/17.
//

#ifndef GEON_ZONAL_OPT_SIGNALHANDLER_H
#define GEON_ZONAL_OPT_SIGNALHANDLER_H

#include <map>

extern std::map< int, bool > signals_called;

extern "C"
{
void signal_handler(int sig);
}


#endif //GEON_ZONAL_OPT_SIGNALHANDLER_H
