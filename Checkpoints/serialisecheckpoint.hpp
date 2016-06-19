#ifndef SERIALISECHECKPOINT_HPP
#define SERIALISECHECKPOINT_HPP

#include "../Checkpoint.hpp"
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/filesystem.hpp>

class SerialiseCheckpoint : public CheckpointBase
{

    int gen_frequency;
    int gen_number;
    NSGAII & nsgaii_engine;
    boost::filesystem::path save_path;

public:
    SerialiseCheckpoint(int _gen_frequency, NSGAII & _nsgaii_engine, boost::filesystem::path _save_path)
    : gen_frequency(_gen_frequency), gen_number(0), nsgaii_engine(_nsgaii_engine), save_path(_save_path)
    {

    }

    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number % gen_frequency == 0)
        {
            boost::filesystem::path save_file = save_path / ("nsgaii_gen" + std::to_string(gen_number) + ".xml");
            std::ofstream ofs(save_file.c_str());
            assert(ofs.good());
            boost::archive::xml_oarchive oa(ofs);
            oa << BOOST_SERIALIZATION_NVP(nsgaii_engine);
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

#endif // SERIALISECHECKPOINT_HPP
