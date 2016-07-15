//
//  ParallelEvaluator.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 8/12/2015.
//
//

#ifndef ParallelEvaluator_h
#define ParallelEvaluator_h

#include <boost/mpi.hpp>
#include "Evaluation.hpp"
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/date_time.hpp>
#include <functional>


class ParallelEvaluatorBase
{
public:
    enum Log{OFF, LVL1, LVL2, LVL3};
protected:
    boost::mpi::environment & mpi_env;
    boost::mpi::communicator & world;
    ProblemDefinitions & problem_defs;
    int number_processes;
    int number_clients;
    std::pair<std::vector<double>, std::vector<int> > decision_vars;
    std::pair<std::vector<double>, std::vector<double> > objs_and_constraints;
    boost::mpi::content dv_c;
    boost::mpi::content oc_c;
    int max_tag;
    Log do_log;
    std::reference_wrapper<std::ostream> log_stream;
    
public:
    ParallelEvaluatorBase(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitions & _problem_defs)
        : mpi_env(_mpi_env), world(_world), problem_defs(_problem_defs), number_processes(world.size()), number_clients(number_processes - 1), max_tag(mpi_env.max_tag()), do_log(OFF), log_stream(std::cout)
    {

    }

    void
    log(Log _val = LVL1, std::ostream & _stream = std::cout)
    {
        do_log = _val;
        if (do_log > OFF)
        {
            log_stream = _stream;
        }
    }

};


class ParallelEvaluatePopServer : public ParallelEvaluatorBase, public EvaluatePopulationBase
{
    
public:
    ParallelEvaluatePopServer(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitions & _problem_defs)
    : ParallelEvaluatorBase(_mpi_env, _world, _problem_defs)
    {
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        decision_vars = std::pair<std::vector<double>, std::vector<int> >
            (   std::piecewise_construct,
                std::forward_as_tuple(std::vector<double>(problem_defs.real_lowerbounds.size(), 0.0)),
                std::forward_as_tuple(std::vector<int>(problem_defs.int_lowerbounds.size(), 0))
            );
        
        objs_and_constraints = std::pair<std::vector<double>, std::vector<double> >
            (   std::piecewise_construct,
         std::forward_as_tuple(std::vector<double>(problem_defs.minimise_or_maximise.size(), 0.0)),
         std::forward_as_tuple(std::vector<double>(problem_defs.number_constraints, 0.0))
         );
        
        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);
        
        dv_c = boost::mpi::get_content(decision_vars);
        oc_c = boost::mpi::get_content(objs_and_constraints);
    }
    
    ~ParallelEvaluatePopServer()
    {
        // Send signal to slaves to indicate shutdown.
        for (int i = 0; i < number_clients; ++i)
        {
            int client_id = i + 1;
            world.send(client_id, max_tag, dv_c);
        }
    }
    
    void
    operator()(PopulationSPtr population)
    {
        if (do_log > OFF) log_stream.get() << "Evaluating population with size: " << population->size() << "\n";

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag - 1))
        {
            if (do_log > OFF) log_stream.get() << "problem: max tag too small, population too large for mpi\n";
        }
        
        int individual = 0;
        std::vector<boost::mpi::request> reqs_out(number_clients);
        for (; individual < number_clients; ++individual)
        {
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = individual + 1;
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << "sending to " << client_id << " individual " << individual << " with " << decision_vars.first[0] << " " << decision_vars.first[1] << std::endl;
            world.send(client_id, individual, dv_c);
        }
//        mpi::wait_all(reqs_out.begin(), reqs_out.end());
        
        while (individual < population->populationSize())
        {
            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << "received from " << s.source() << " individual " << individual << " with " << objs_and_constraints.first[0] << " " << objs_and_constraints.first[1] << std::endl;
            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
            
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << "sending to " << s.source() << " individual " << individual << " with " << decision_vars.first[0] << " " << decision_vars.first[1] << std::endl;
            world.send(s.source(), individual, dv_c);

            ++individual;
        }
        
        for (int i = 0; i < number_clients; ++i)
        {
            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
        }
    
    }
};



class ParallelEvaluatePopClient : public ParallelEvaluatorBase
{
    ObjectivesAndConstraintsBase & eval;
    
public:
    ParallelEvaluatePopClient(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitions & _problem_defs, ObjectivesAndConstraintsBase & _eval)
    : ParallelEvaluatorBase(_mpi_env, _world, _problem_defs), eval(_eval)
    {
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);
        
        dv_c = boost::mpi::get_content(decision_vars);
        oc_c = boost::mpi::get_content(objs_and_constraints);
        
    }
    
    void
    operator()()
    {
        
        bool do_continue = true;
        while (do_continue)
        {
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << "waiting to receive" << std::endl;
            boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received " << decision_vars.first[0] << " " << decision_vars.first[1] << " for individual " << s.tag() << std::endl;
            if (s.tag() == max_tag)
            {
                do_continue = false;
            }
            else
            {
                //calc objective
                objs_and_constraints = eval(decision_vars.first, decision_vars.second);
                world.send(0, s.tag(), oc_c);
            }
        }
    }
};



#endif /* ParallelEvaluator_h */
