//
//  ParallelEvaluatorServer.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 4/12/2015.
//
//

#ifndef ParallelEvaluatorServer_h
#define ParallelEvaluatorServer_h

#include <boost/mpi.hpp>


class ParallelEvaluatePopServer :
{
private:
    boost::mpi::environment & mpi_env;
    boost::mpi::communicator & world;
    ProblemDefinitions & problem_defs;
    
public:
    ParallelEvaluatePopServer(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world)
    : mpi_env(_mpi_env), world(_world), 
    {
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        std::pair<std::vector<double>, std::vector<int> > decision_vars
            (   std::piecewise_construct,
                std::forward_as_tuple(std::vector<double>(problem_defs.real_lowerbounds.size(), 0.0)),
                std::forward_as_tuple(std::vector<double>(problem_defs.int_lowerbounds.size(), 0))
             );
        boost::mpi::broadcast(world, mpi::skeleton(decision_vars))
    }
    
    void
    operator()(PopulationSPtr population)
    {
        
        
        std::vector<mpi::request> reqs_out(population->populationSize());
        std::vector<mpi::request> reqs_in(population->populationSize());
        for (int i = 0; i < population->populationSize(); ++i)
        {
            reqs_out[i] = boost::mpi::isend();
        }
        for (int i = 0; i < population->populationSize(); ++i)
        {
            std::vector<double> objectives;
            std::vector<double> constraints;
            reqs_in[i] = boost::mpi::irecv();
            ind.setObjectives(objectives);
            ind.setConstraints(constraints);
        }
    }
};

#endif /* ParallelEvaluatorServer_h */
