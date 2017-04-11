//
//  Checkpoint.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 25/11/2015.
//
//

#ifndef Checkpoint_h
#define Checkpoint_h


#include <boost/foreach.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/shared_ptr.hpp>
#include "Population.hpp"


class CheckpointBase
{
public:
    virtual bool operator() (PopulationSPtr population) = 0;
};

BOOST_SERIALIZATION_ASSUME_ABSTRACT( CheckpointBase )

typedef boost::shared_ptr<CheckpointBase> CheckpointBaseSPtr;

class DummyCheckpoint : public CheckpointBase
{
public:
    bool
    operator()(PopulationSPtr population)
    {
        return true;
    }
};

class Checkpoints
{
private:
    std::vector<CheckpointBaseSPtr> my_checkpoints;
//    std::vector<CheckpointBase &> my_checkpoints;

public:
    Checkpoints()
    {

    }

    void
    addCheckpoint(CheckpointBaseSPtr checkpoint_2_add)
    {
        my_checkpoints.push_back(checkpoint_2_add);
    }

    bool
    operator()(PopulationSPtr population)
    {
        bool do_continue = true;
        BOOST_FOREACH(CheckpointBaseSPtr chckpnt, my_checkpoints)
        {
            do_continue = (do_continue && chckpnt->operator ()(population));
        }
        return (do_continue);
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(my_checkpoints);
    }


};





#endif /* Checkpoint_h */
