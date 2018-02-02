//
//  ParallelEvaluator.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 8/12/2015.
//
//

#ifndef ParallelEvaluator_h
#define ParallelEvaluator_h

#include <ctime>
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
    ProblemDefinitionsSPtr problem_defs;
    int number_processes;
    int number_clients;
    std::pair<std::vector<double>, std::vector<int> > decision_vars;
    std::pair<std::vector<double>, std::vector<double> > objs_and_constraints;
    std::pair<std::vector<double>, std::vector<double> > worst_objs_and_constraints;
    boost::mpi::content dv_c;
    boost::mpi::content oc_c;
    int max_tag;
    Log do_log;
    std::reference_wrapper<std::ostream> log_stream;
    std::time_t timeout_time;
    const std::string NO_SAVE = "no_save";
    const std::string TERMINATE = "terminate";
    
public:
    ParallelEvaluatorBase(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs)
        : mpi_env(_mpi_env), world(_world), problem_defs(_problem_defs), number_processes(world.size()), number_clients(number_processes - 1), max_tag(mpi_env.max_tag()), do_log(OFF), log_stream(std::cout)
    {

        worst_objs_and_constraints.first.resize(problem_defs->minimise_or_maximise.size());
        for (int i = 0; i < worst_objs_and_constraints.first.size(); ++i)
        {
            if (problem_defs->minimise_or_maximise.at(i) == MINIMISATION ) worst_objs_and_constraints.first.at(i) = std::numeric_limits<double>::max();
            if (problem_defs->minimise_or_maximise.at(i) == MAXIMISATION ) worst_objs_and_constraints.first.at(i) = std::numeric_limits<double>::min();
        }
        worst_objs_and_constraints.second.resize(problem_defs->number_constraints);
        BOOST_FOREACH(double & constraint, worst_objs_and_constraints.second)
        {
            constraint = std::numeric_limits<double>::max();
        }

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

namespace{


    inline std::ostream& operator << (std::ostream& os, const std::pair<std::vector<double>, std::vector<int> >& v)
    {
        os << "[";
        for (std::vector<double>::const_iterator ii = v.first.begin(); ii != v.first.end(); ++ii)
        {
            os << " " << *ii;
        }
        for (std::vector<int>::const_iterator ii = v.second.begin(); ii != v.second.end(); ++ii)
        {
            os << " " << *ii;
        }
        os << " ]";
        return os;
    }


    inline std::ostream& operator << (std::ostream& os, const std::pair<std::vector<double>, std::vector<double> >& v)
    {
        os << "[";
        for (std::vector<double>::const_iterator ii = v.first.begin(); ii != v.first.end(); ++ii)
        {
            os << " " << *ii;
        }
        for (std::vector<double>::const_iterator ii = v.second.begin(); ii != v.second.end(); ++ii)
        {
            os << " " << *ii;
        }
        os << " ]";
        return os;
    }

}



class ParallelEvaluatePopServer : public ParallelEvaluatorBase, public EvaluatePopulationBase
{
    
public:
    ParallelEvaluatePopServer(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs)
    : ParallelEvaluatorBase(_mpi_env, _world, _problem_defs)
    {
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        decision_vars = std::pair<std::vector<double>, std::vector<int> >
            (   std::piecewise_construct,
                std::forward_as_tuple(std::vector<double>(problem_defs->real_lowerbounds.size(), 0.0)),
                std::forward_as_tuple(std::vector<int>(problem_defs->int_lowerbounds.size(), 0))
            );

        objs_and_constraints = std::pair<std::vector<double>, std::vector<double> >
            (   std::piecewise_construct,
         std::forward_as_tuple(std::vector<double>(problem_defs->minimise_or_maximise.size(), 0.0)),
         std::forward_as_tuple(std::vector<double>(problem_defs->number_constraints, 0.0))
         );

        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);

        dv_c = boost::mpi::get_content(decision_vars);
        oc_c = boost::mpi::get_content(objs_and_constraints);
    }
    
    ~ParallelEvaluatePopServer()
    {
        // Send signal to slaves to indicate shutdown.
//        std::cout << "In destructor" << std::endl;
        if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending terminate" << std::endl;
        std::string terminate = TERMINATE;
        boost::mpi::broadcast(world, terminate,0);
    }

    void
    evalPop(PopulationSPtr population, boost::filesystem::path save_dir)
    {
        std::string save_dir_s = save_dir.string();
        boost::mpi::broadcast(world, save_dir_s,0);

        if (do_log > OFF) log_stream.get() << "Evaluating population with size: " << population->size() << std::endl;

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag - 1))
        {
            if (do_log > OFF) log_stream.get() << "problem: max tag too small, population too large for mpi" << std::endl;
        }

        int individual = 0;
        std::vector<boost::mpi::request> reqs_out(number_clients);
        if (do_log > OFF) log_stream.get() << "Evaluating population using " << number_clients << std::endl;
        int num_initial_jobs = number_clients;
        if (num_initial_jobs > population->populationSize()) num_initial_jobs = population->populationSize();
        for (; individual < num_initial_jobs; ++individual)
        {
            if (do_log > OFF) log_stream.get() << world.rank() << ": " << "Evaluating individual " << individual << std::endl;
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = individual + 1;
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars << std::endl;
            world.send(client_id, individual, dv_c);
        }
//        mpi::wait_all(reqs_out.begin(), reqs_out.end());

        while (individual < population->populationSize())
        {
            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);

            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << s.source() << " individual " << individual << " with " << decision_vars << std::endl;
            world.send(s.source(), individual, dv_c);

            ++individual;
        }

        for (int i = 0; i < number_clients; ++i)
        {
            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);


//            world.send(s.source(), max_tag, dv_c);
        }

        for (int i = 0; i < number_clients; ++i)
        {
            int client_id = i + 1;
//            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
            world.isend(client_id, max_tag, dv_c);
        }
    }
    
    void
    operator()(PopulationSPtr population)
    {
        this->evalPop(population, NO_SAVE);
    }

    void
    operator()(PopulationSPtr population, boost::filesystem::path & save_dir)
    {
        this->evalPop(population, save_dir);
    }
};

class ParallelEvaluatePopServerNonBlocking : public ParallelEvaluatorBase, public EvaluatePopulationBase
{

public:
    ParallelEvaluatePopServerNonBlocking(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs, std::time_t _timeout_time = 1800)
    : ParallelEvaluatorBase(_mpi_env, _world, _problem_defs)
    {
        this->timeout_time = _timeout_time;

        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        decision_vars = std::pair<std::vector<double>, std::vector<int> >
            (   std::piecewise_construct,
                std::forward_as_tuple(std::vector<double>(problem_defs->real_lowerbounds.size(), 0.0)),
                std::forward_as_tuple(std::vector<int>(problem_defs->int_lowerbounds.size(), 0))
            );
        
        objs_and_constraints = std::pair<std::vector<double>, std::vector<double> >
        (   std::piecewise_construct,
         std::forward_as_tuple(std::vector<double>(problem_defs->minimise_or_maximise.size(), 0.0)),
         std::forward_as_tuple(std::vector<double>(problem_defs->number_constraints, 0.0))
         );

        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);

        dv_c = boost::mpi::get_content(decision_vars);
        oc_c = boost::mpi::get_content(objs_and_constraints);
    }

    ~ParallelEvaluatePopServerNonBlocking()
    {
        // Send signal to slaves to indicate shutdown.
//        std::cout << "In destructor" << std::endl;
        if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending terminate" << std::endl;
        std::string terminate = TERMINATE;
        boost::mpi::broadcast(world, terminate,0);
    }

    void
    evalPop(PopulationSPtr population, boost::filesystem::path save_dir)
    {

        std::string save_dir_s = save_dir.string();
        boost::mpi::broadcast(world, save_dir_s,0);

        if (do_log > OFF) log_stream.get() << "Evaluating population with size: " << population->size() << std::endl;

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag - 1))
        {
            if (do_log > OFF) log_stream.get() << "problem: max tag too small, population too large for mpi" << std::endl;
        }

        typedef std::pair<const int, boost::mpi::request> JobRunningT;
        std::map<int, boost::mpi::request> jobs_running;

        int individual = 0;
//        std::vector<boost::mpi::request> reqs_out(number_clients);
        int num_initial_jobs = number_clients;
        if (num_initial_jobs > population->populationSize()) num_initial_jobs = population->populationSize();
        for (; individual < num_initial_jobs; ++individual)
        {
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = individual + 1;
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars << std::endl;
            jobs_running.insert(std::make_pair(individual, world.isend(client_id, individual, dv_c)));
        }

        // keep jobs out there for slave eficiency (so queue an additional job for each slave)
        int num_additional_jobs = number_clients;
        if ((num_additional_jobs + num_initial_jobs) > population->populationSize()) num_additional_jobs = population->populationSize() - num_initial_jobs;
        for (int i = 0; i < num_additional_jobs; ++i)
        {
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = individual + 1 - num_initial_jobs;
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars << std::endl;
            jobs_running.insert(std::make_pair(individual, world.isend(client_id, individual, dv_c)));
            ++individual;
        }

//        mpi::wait_all(reqs_out.begin(), reqs_out.end());
        boost::mpi::status s;

        while (individual < population->populationSize())
        {
            //start a receive request, non-blocking
            boost::mpi::request r = world.irecv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            //get start time
            std::time_t start_time = std::time(NULL);
            // test if we have made a receive.
            boost::optional<boost::mpi::status> osr = r.test();
            if (!osr)
            {
                //loop until we have received, or taken too long
                while (!osr && (std::difftime(std::time(NULL),start_time) < this->timeout_time))
                {
                    //wait a bit.
                    osr = r.test();
                }
            }

            //By now we either have received the data, or taken too long, so...
            if (!osr)
            {
                //we must have timed out
                r.cancel();
            }
            else
            {
                s = osr.get();
                if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
                (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
                (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
                boost::optional<boost::mpi::status> oss = jobs_running[s.tag()].test();
                if (!oss)
                {
                    if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending failed. But we have a receive. Strange " << std::endl;
                    jobs_running[s.tag()].cancel();
                }
                jobs_running.erase(s.tag());

                // Send out new job.
                decision_vars.first = (*population)[individual]->getRealDVVector();
                decision_vars.second = (*population)[individual]->getIntDVVector();
                if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << s.source() << " individual " << individual << " with " << decision_vars << std::endl;
                jobs_running.insert(std::make_pair(individual, world.isend(s.source(), individual, dv_c)));

                ++individual;
            }




        }

        // test for sanity
        if ((num_additional_jobs + num_initial_jobs) != jobs_running.size())
        {
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " Logic error: job accounting wrong or there were incomplete jobs" << std::endl;
        }

        for (int i = 0; i < (num_initial_jobs + num_additional_jobs); ++i)
        {
            //start a receive request, non-blocking
            boost::mpi::request r = world.irecv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            //get start time
            std::time_t start_time = std::time(NULL);
            // test if we have made a receive.
            boost::optional<boost::mpi::status> os = r.test();
            if (!os)
            {
                //loop until we have received, or taken too long
                while (!os && (std::difftime(std::time(NULL),start_time) < this->timeout_time))
                {
                    //wait a bit.
                    os = r.test();
                }
            }

            //By now we either have received the data, or taken too long, so...
            if (!os)
            {
                //we must have timed out
                r.cancel();
            }
            else
            {
                boost::mpi::status s = os.get();
                if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
                (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
                (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
                boost::optional<boost::mpi::status> oss = jobs_running[s.tag()].test();
                if (!oss)
                {
                    if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending failed. But we have a receive. Strange " << std::endl;
                    jobs_running[s.tag()].cancel();
                }
                jobs_running.erase(s.tag());
            }

        }

        BOOST_FOREACH(JobRunningT & job, jobs_running)
                    {
                        // Failed jobs.
                        // set decision variables to something not so good.
                        boost::optional<boost::mpi::status> oss = job.second.test();
                        if (!oss)
                        {
                            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending failed. But we have a receive. Strange " << std::endl;
                            job.second.cancel();
                        }
                        (*population)[job.first]->setObjectives(this->worst_objs_and_constraints.first);
                        (*population)[job.first]->setConstraints(this->worst_objs_and_constraints.second);
                    }

        for (int i = 0; i < number_clients; ++i)
        {
            int client_id = i + 1;
//            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
            if (do_log > OFF) log_stream.get() << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
            world.isend(client_id, max_tag, dv_c);
        }

    }

    void
    operator()(PopulationSPtr population)
    {
       this->evalPop(population, NO_SAVE);
    }

    void
    operator()(PopulationSPtr population, boost::filesystem::path & save_dir)
    {
        this->evalPop(population, save_dir);

    }
};



class ParallelEvaluatePopClientNonBlocking : public ParallelEvaluatorBase
{
    ObjectivesAndConstraintsBase & eval;

public:
    ParallelEvaluatePopClientNonBlocking(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
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
        bool in_generation = true;
        boost::mpi::request rq;
        bool first_time = true;
        std::string save_dir_s;
        boost::filesystem::path save_dir;
        bool do_save;
        
        while (do_continue)
        {
            boost::mpi::broadcast(world, save_dir_s,0);
            if (save_dir_s == NO_SAVE)
            {
                do_save = false;
            }
            else if (save_dir_s == TERMINATE)
            {
                in_generation = false;
                do_continue = false;
                if (do_log > OFF)
                    log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Terminating" << std::endl;
                break;
            }
            else
            {
                do_save = true;
                save_dir = save_dir_s;
                in_generation = true;
            }

            while (in_generation)
            {
                if (do_log > OFF)
                    log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " waiting to receive" << std::endl;
                boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
                if (do_log > OFF)
                    log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " received " << decision_vars << " for individual " << s.tag() << std::endl;
                if (s.tag() == max_tag)
                {
                    in_generation = false;
                    if (do_log > OFF)
                        log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                         << " End of generation" << std::endl;
                }
                else
                {
                    //calc objective
                    if (!do_save)
                    {
                        objs_and_constraints = eval(decision_vars.first, decision_vars.second);
                    }
                    else
                    {
                        boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(s.tag()));
                        if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
                        objs_and_constraints = eval(decision_vars.first, decision_vars.second, save_ind_dir);
                    }

                    if (do_log > OFF)
                        log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                         << " sending " << objs_and_constraints << " for individual " << s.tag()
                                         << std::endl;
                    if (!first_time)
                    {
                        rq.wait();

                    }
                    else
                    {
                        first_time = false;
                    }
                    rq = world.isend(0, s.tag(), oc_c);

                }
            }

        }
    }
};

class ParallelEvaluatePopClient : public ParallelEvaluatorBase
{
    ObjectivesAndConstraintsBase & eval;

public:
    ParallelEvaluatePopClient(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
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
        bool in_generation = true;
        boost::mpi::request rq;
        bool first_time = true;
        std::string save_dir_s;
        boost::filesystem::path save_dir;
        bool do_save;

        while (do_continue)
        {
            boost::mpi::broadcast(world, save_dir_s,0);
            if (save_dir == "no_save")
            {
                do_save = false;
            }
            else if (save_dir == "terminate")
            {
                in_generation = false;
                do_continue = false;
                if (do_log > OFF)
                    log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Terminating" << std::endl;
                break;
            }
            else
            {
                do_save = true;
                save_dir = save_dir_s;
                in_generation = true;
            }
            while (in_generation)
            {
                if (do_log > OFF)
                    log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " waiting to receive" << std::endl;
                boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
                if (do_log > OFF)
                    log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " received " << decision_vars << " for individual " << s.tag() << std::endl;
                if (s.tag() == max_tag)
                {
                    in_generation = false;
                    if (do_log > OFF)
                        log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                         << " End of generation" << std::endl;
                }
                else
                {
                    //calc objective
                    if (!do_save)
                    {
                        objs_and_constraints = eval(decision_vars.first, decision_vars.second);
                    }
                    else
                    {
                        boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(s.tag()));
                        if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
                        objs_and_constraints = eval(decision_vars.first, decision_vars.second, save_ind_dir);
                    }

                    if (do_log > OFF)
                        log_stream.get() << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                         << " sending " << objs_and_constraints << " for individual " << s.tag()
                                         << std::endl;
                    if (!first_time)
                    {
                        rq.wait();

                    }
                    else
                    {
                        first_time = false;
                    }
                    world.send(0, s.tag(), oc_c);

                }
            }

        }
    }
};




#endif /* ParallelEvaluator_h */
