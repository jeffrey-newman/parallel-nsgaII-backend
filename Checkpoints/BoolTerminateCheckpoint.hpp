//
// Created by a1091793 on 8/11/17.
//

#ifndef GEON_OPT_BOOLTERMINATECHECKPOINT_H
#define GEON_OPT_BOOLTERMINATECHECKPOINT_H


#include <Checkpoint.hpp>

class BoolTerminateCheckpoint : public CheckpointBase
{
private:
    bool & do_terminate;

public:
    BoolTerminateCheckpoint(bool& _do_terminate):
            do_terminate(_do_terminate)
    {

    }

    bool
    operator()(PopulationSPtr population)
    {
        if (do_terminate) return false;
        return true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(do_terminate);
    }
};


#endif //GEON_OPT_BOOLTERMINATECHECKPOINT_H
