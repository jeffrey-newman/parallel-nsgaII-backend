#ifndef METRICBASE_HPP
#define METRICBASE_HPP

#include "../Population.hpp"
#include "../Checkpoint.hpp"


class MetricBase : public CheckpointBase
{
public:
    virtual bool
    operator ()(PopulationSPtr pop) =0;

    virtual double
    getVal() = 0;
    
    virtual ~MetricBase()
    {
        
    }

};

typedef boost::shared_ptr<MetricBase> MetricBaseSPtr;

#endif // METRICBASE_HPP
