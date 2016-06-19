//
//  Checkpoint.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 25/11/2015.
//
//

#ifndef Checkpoint_h
#define Checkpoint_h

#include <boost/serialization/nvp.hpp>


class CheckpointBase
{
    virtual bool operator() (PopulationSPtr population) = 0;
};

class DummyCheckpoint : public CheckpointBase
{
public:
    bool
    operator()(PopulationSPtr population)
    {
        return true;
    }
};

class MaxGenCheckpoint : public CheckpointBase
{
    int max_gen;
    int gen_number;
    
public:
    Checkpoint(int _max_gen)
    : max_gen(_max_gen), gen_number(0)
    {
        
    }
    
    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number > max_gen)
        {
            return false;
        }
        return true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(max_gen);
            ar & BOOST_SERIALIZATION_NVP(gen_number);
    }
};

class SerialiseCheckpoint : public CheckpointBase
{
    int gen_frequency;
    int gen_number;
    NSGAII

public:
    SerialiseCheckpoint(int _max_gen)
    : max_gen(_max_gen), gen_number(0)
    {

    }

    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number % gen_frequency == 0)
        {

        }
        return true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(gen_frequency);
            ar & BOOST_SERIALIZATION_NVP(gen_number);
    }
};

#endif /* Checkpoint_h */
