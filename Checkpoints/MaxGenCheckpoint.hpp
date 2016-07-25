#ifndef MAXGENCHECKPOINT_HPP
#define MAXGENCHECKPOINT_HPP

#include "../Checkpoint.hpp"
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>

class MaxGenCheckpoint : public CheckpointBase
{
    int max_gen;
    int gen_number;

public:

    MaxGenCheckpoint(int _max_gen)
    : max_gen(_max_gen), gen_number(0)
    {

    }

    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
//        std::cout << gen_number << ": " << max_gen << std::endl;
        if (gen_number >= max_gen)
        {
            std::cout << "Terminating. Exceeding maximum generation. Generation number: " << (gen_number) << std::endl;
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

//BOOST_CLASS_EXPORT_KEY(MaxGenCheckpoint);
//BOOST_CLASS_EXPORT_IMPLEMENT(MaxGenCheckpoint);

#endif // MAXGENCHECKPOINT_HPP
