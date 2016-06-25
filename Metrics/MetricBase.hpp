#ifndef METRICBASE_HPP
#define METRICBASE_HPP

#include "../Population.hpp"
#include "../Checkpoint.hpp"


class MetricBase : CheckpointBase
{
public:
    virtual bool
    operator ()(PopulationSPtr pop) =0;

    virtual double
    getVal() = 0;

};

#endif // METRICBASE_HPP
