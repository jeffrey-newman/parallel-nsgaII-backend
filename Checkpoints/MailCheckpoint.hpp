#ifndef MAILCHECKPOINT_HPP
#define MAILCHECKPOINT_HPP

#include <vector>
#include <string>
#include <sstream>
#include <boost/serialization/vector.hpp>
#include "../Checkpoint.hpp"
#include "../Population.hpp"
#include "Metrics/MetricBase.hpp"

class MailCheckpoint : public CheckpointBase
{

    int gen_frequency;
    int gen_number;
    std::vector<std::string> addresses;
    MetricBaseSPtr metric;
    std::string disp_name;

public:
    MailCheckpoint(int _gen_frequency, MetricBaseSPtr _metric, std::string _disp_name)
    : gen_frequency(_gen_frequency), gen_number(0), metric(_metric), disp_name(_disp_name)
    {

    }

    void
    addAddress(std::string & address)
    {
        addresses.push_back(address);
    }

    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number % gen_frequency == 0)
        {
            std::stringstream command_start;
            command_start << "mail -s \"" << disp_name << "--- Generation " << gen_number << " : " << metric->getVal() << "\" ";
            BOOST_FOREACH(std::string & address, addresses)
            {
                std::stringstream command;
                command << command_start.str() << address << " < /dev/null";
                std::cout << command.str() << std::endl;
                system(command.str().c_str());

            }
        }
        return true;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
            ar & BOOST_SERIALIZATION_NVP(gen_frequency);
            ar & BOOST_SERIALIZATION_NVP(gen_number);
//            ar & BOOST_SERIALIZATION_NVP(metric);
            ar & BOOST_SERIALIZATION_NVP(disp_name);
    }
};




#endif // MAILCHECKPOINT_HPP
