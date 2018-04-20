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
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/date_time.hpp>
#include <functional>
#include <queue>
#include <unordered_map>
#include "Selection.hpp"
#include "Crossover.hpp"
#include "Mutation.hpp"


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
    int do_log;
    int gen_num = 0;
    boost::filesystem::path log_directory;
//    std::ofstream log_stream;
    std::time_t timeout_time;
    const std::string NO_SAVE = "no_save";
    const std::string TERMINATE = "terminate";
    
public:
    ParallelEvaluatorBase(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs)
        : mpi_env(_mpi_env),
          world(_world),
          problem_defs(_problem_defs),
          number_processes(world.size()),
          number_clients(number_processes - 1),
          max_tag(mpi_env.max_tag()),
          do_log(this->OFF)
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
    log(boost::filesystem::path _log_directory, int _val = LVL1)
    {
        this->do_log = _val;
        this->log_directory= _log_directory;
    }

};

class ParallelEvaluatorClientBase : public ParallelEvaluatorBase
{
public:
    ParallelEvaluatorClientBase(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs)
        : ParallelEvaluatorBase(_mpi_env, _world, _problem_defs)
    {
//        std::cout << "receiving broadcast skeleton" << std::endl;
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);

//        std::cout << "Getting content for skeleton" << std::endl;
        dv_c = boost::mpi::get_content(decision_vars);
        oc_c = boost::mpi::get_content(objs_and_constraints);
//        std::cout << "Completed initialisation Client" << std::endl;
    }

    virtual void operator()()=0;
};

class ParallelEvaluatorSeverBase : public ParallelEvaluatorBase, public EvaluatePopulationBase
{
public:
    ParallelEvaluatorSeverBase(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs, std::time_t _timeout_time = 1800)
    : ParallelEvaluatorBase(_mpi_env, _world, _problem_defs)
    {
        this->timeout_time = _timeout_time;

        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient
        //Send skeleton of decision variable to make sending dvs to clients/slaves more efficient

//        std::cout << "Real DVs: " << problem_defs->real_lowerbounds.size() << std::endl;
//        std::cout << "Int DVs: " << problem_defs->int_lowerbounds.size() << std::endl;
//        std::cout << "Objs: " << problem_defs->minimise_or_maximise.size() << std::endl;
//        std::cout << "Constraints: " << problem_defs->number_constraints << std::endl;

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

//        std::cout << "Broadcasting skeleton" << std::endl;
        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);

//        std::cout << "Getting content for skeleton" << std::endl;
        dv_c = boost::mpi::get_content(decision_vars);
        oc_c = boost::mpi::get_content(objs_and_constraints);
//        std::cout << "Completed initialisation Server" << std::endl;
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



class ParallelEvaluatePopServer : public ParallelEvaluatorSeverBase
{
    
public:
    ParallelEvaluatePopServer(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs, std::time_t _timeout_time = 1800)
    : ParallelEvaluatorSeverBase(_mpi_env, _world, _problem_defs)
    {

    }
    
    ~ParallelEvaluatePopServer()
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_eval_server_term.log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        // Send signal to slaves to indicate shutdown.
//        std::cout << "In destructor" << std::endl;
        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending terminate" << std::endl;
        std::string terminate = TERMINATE;
        boost::mpi::broadcast(world, terminate,0);
    }

    void
    evalPop(PopulationSPtr population, boost::filesystem::path save_dir)
    {

        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_eval_server" + std::to_string(this->gen_num) + ".log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " Broadcasting save directory: " << save_dir << std::endl;
        std::string save_dir_s = save_dir.string();
        boost::mpi::broadcast(world, save_dir_s,0);

        if (this->do_log > this->OFF) logging_file << "Evaluating population with size: " << population->size() << std::endl;

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag - 1))
        {
            if (this->do_log > this->OFF) logging_file << "problem: max tag too small, population too large for mpi" << std::endl;
        }

        int individual = 0;
        std::vector<boost::mpi::request> reqs_out(number_clients);
        if (this->do_log > this->OFF) logging_file << "Evaluating population using " << number_clients << std::endl;
        int num_initial_jobs = number_clients;
        if (num_initial_jobs > population->populationSize()) num_initial_jobs = population->populationSize();
        for (; individual < num_initial_jobs; ++individual)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " << "Evaluating individual " << individual << std::endl;
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = individual + 1;
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with "  << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVs: " << decision_vars << std::endl;
            world.send(client_id, individual, dv_c);
        }
//        mpi::wait_all(reqs_out.begin(), reqs_out.end());

        while (individual < population->populationSize())
        {
            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);

            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << s.source() << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVs: " << decision_vars << std::endl;
            world.send(s.source(), individual, dv_c);

            ++individual;
        }

        for (int i = 0; i < number_clients; ++i)
        {
            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);


//            world.send(s.source(), max_tag, dv_c);
        }

        for (int i = 0; i < number_clients; ++i)
        {
            int client_id = i + 1;
//            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
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

class ParallelEvaluatePopServerNonBlocking : public ParallelEvaluatorSeverBase
{

public:
    ParallelEvaluatePopServerNonBlocking(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs, std::time_t _timeout_time = 1800)
    : ParallelEvaluatorSeverBase(_mpi_env, _world, _problem_defs)
    {

    }

    ~ParallelEvaluatePopServerNonBlocking()
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_pop_server_nonblock_term.log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }
        // Send signal to slaves to indicate shutdown.
//        std::cout << "In destructor" << std::endl;
        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending terminate" << std::endl;
        std::string terminate = TERMINATE;
        boost::mpi::broadcast(world, terminate,0);

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }
    }

    void
    evalPop(PopulationSPtr population, boost::filesystem::path save_dir)
    {

        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_server_nonblock_gen" + std::to_string(++this->gen_num) + ".log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " Broadcasting save directory: " << save_dir << std::endl;
        std::string save_dir_s = save_dir.string();
        boost::mpi::broadcast(world, save_dir_s,0);

        if (this->do_log > this->OFF) logging_file << "Evaluating population with size: " << population->size() << std::endl;

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag - 1))
        {
            if (this->do_log > this->OFF) logging_file << "problem: max tag too small, population too large for mpi" << std::endl;
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
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
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
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
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
                if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
                (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
                (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
                boost::optional<boost::mpi::status> oss = jobs_running[s.tag()].test();
                if (!oss)
                {
                    if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending failed. But we have a receive. Strange " << std::endl;
                    jobs_running[s.tag()].cancel();
                }
                jobs_running.erase(s.tag());

                // Send out new job.
                decision_vars.first = (*population)[individual]->getRealDVVector();
                decision_vars.second = (*population)[individual]->getIntDVVector();
                if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << s.source() << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
                jobs_running.insert(std::make_pair(individual, world.isend(s.source(), individual, dv_c)));

                ++individual;
            }




        }

        // test for sanity
        if ((num_additional_jobs + num_initial_jobs) != jobs_running.size())
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " Logic error: job accounting wrong or there were incomplete jobs" << std::endl;
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
                if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
                (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
                (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
                boost::optional<boost::mpi::status> oss = jobs_running[s.tag()].test();
                if (!oss)
                {
                    if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending failed. But we have a receive. Strange " << std::endl;
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
                            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending failed. But we have a receive. Strange " << std::endl;
                            job.second.cancel();
                        }
                        (*population)[job.first]->setObjectives(this->worst_objs_and_constraints.first);
                        (*population)[job.first]->setConstraints(this->worst_objs_and_constraints.second);
                    }

        for (int i = 0; i < number_clients; ++i)
        {
            int client_id = i + 1;
//            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
            world.isend(client_id, max_tag, dv_c);
        }

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }

    }

    void
    operator()(PopulationSPtr population)
    {
       this->evalPop(population, NO_SAVE);
    }

    void
    operator()(PopulationSPtr population, const boost::filesystem::path & save_dir)
    {
        this->evalPop(population, save_dir);

    }
};



class ParallelEvaluatePopClientNonBlocking : public ParallelEvaluatorClientBase
{
    ObjectivesAndConstraintsBase & eval;

public:
    ParallelEvaluatePopClientNonBlocking(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
    : ParallelEvaluatorClientBase(_mpi_env, _world, _problem_defs), eval(_eval)
    {

    }

    void
    operator()()
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_client" + std::to_string(world.rank()) + "_nonblock" + std::to_string(++this->gen_num) + std::string(".log");
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        bool do_continue = true;
        bool in_generation = true;
        boost::mpi::request rq;
        bool first_time = true;
        std::string save_dir_s;
        boost::filesystem::path save_dir;
        bool do_save;
        
        while (do_continue)
        {
            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " Waiting to receive save-dir" << std::endl;
            boost::mpi::broadcast(world, save_dir_s,0);
            if (save_dir_s == NO_SAVE)
            {
                do_save = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Not saving evaluation to file" << std::endl;
                in_generation = true;
            }
            else if (save_dir_s == TERMINATE)
            {
                in_generation = false;
                do_continue = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Terminating" << std::endl;
                break;
            }
            else
            {
                do_save = true;
                save_dir = save_dir_s;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Saving evaluation here: " << save_dir_s << std::endl;
                in_generation = true;
            }

            while (in_generation)
            {
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " waiting to receive" << std::endl;
                boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " received " << decision_vars << " for individual " << s.tag() << std::endl;
                if (s.tag() == max_tag)
                {
                    in_generation = false;
                    if (this->do_log > this->OFF)
                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
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

                    if (this->do_log > this->OFF)
                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                         << " sending " << objs_and_constraints << " for individual " << s.tag();
                    logging_file.flush();
//                                         << std::endl;
                    if (!first_time)
                    {
                        rq.wait();

                    }
                    else
                    {
                        first_time = false;
                    }
                    rq = world.isend(0, s.tag(), oc_c);
                    if (this->do_log > this->OFF)
                        logging_file << "; now sent"  << std::endl;

                }
            }

        }

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }
    }
};

class ParallelEvaluatePopClient : public ParallelEvaluatorClientBase
{
    ObjectivesAndConstraintsBase & eval;

public:
    ParallelEvaluatePopClient(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
        : ParallelEvaluatorClientBase(_mpi_env, _world, _problem_defs), eval(_eval)
    {

    }

    void
    operator()()
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_client" + std::to_string(world.rank()) + std::to_string(++this->gen_num) + std::string(".log");
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }


        bool do_continue = true;
        bool in_generation = true;
//        boost::mpi::request rq;
//        bool first_time = true;
        std::string save_dir_s;
        boost::filesystem::path save_dir;
        bool do_save;

        while (do_continue)
        {
            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " Waiting to receive save-dir" << std::endl;
            boost::mpi::broadcast(world, save_dir_s,0);
            if (save_dir_s == "no_save")
            {
                do_save = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Not saving evaluation to file" << std::endl;
                in_generation = true;
            }
            else if (save_dir_s == "terminate")
            {
                in_generation = false;
                do_continue = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Terminating" << std::endl;
                break;
            }
            else
            {
                do_save = true;
                save_dir = save_dir_s;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Saving evaluation here: " << save_dir_s << std::endl;
                in_generation = true;
            }
            while (in_generation)
            {
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " waiting to receive" << std::endl;
                boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " received " << decision_vars << " for individual " << s.tag() << std::endl;
                if (s.tag() == max_tag)
                {
                    in_generation = false;
                    if (this->do_log > this->OFF)
                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
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

                    if (this->do_log > this->OFF)
                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                         << " sending " << objs_and_constraints << " for individual " << s.tag()
                                         << std::endl;

                    world.send(0, s.tag(), oc_c);

                }
            }

        }

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }
    }
};

template <typename RNG_PE = std::mt19937>
class ParallelEvaluatePopServerNonBlockingContinuousEvolution : public ParallelEvaluatorSeverBase
{

private:
    std::queue<IndividualSPtr> jobs;
    std::unordered_map<int, std::pair<boost::mpi::request, IndividualSPtr> > jobs_running;
    const int CLIENT_QUEUE_SIZE = 2;
//    const bool addOffspring2Selection = true;
    bool add_offspring_2_mating_pool = true;
    int generational_reproduction_size;
    TournamentSelectionContinuousEvolution<RNG_PE> & selection;
    CombinedRealIntCrossover<RNG_PE> & crossover;
    CombinedRealIntMutation<RNG_PE> & mutation;
    int job_num = 0;
    int gen_num = 0;
    int breed_and_eval_count = 0;
    int eval_and_save_count = 0;

public:
    ParallelEvaluatePopServerNonBlockingContinuousEvolution(boost::mpi::environment & _mpi_env,
                                                            boost::mpi::communicator & _world,
                                                            ProblemDefinitionsSPtr _problem_defs,
                                                            TournamentSelectionContinuousEvolution<RNG_PE> & _selection,
                                                            CombinedRealIntCrossover<RNG_PE> & _crossover,
                                                            CombinedRealIntMutation<RNG_PE> & _mutation,
                                                            std::time_t _timeout_time = 1800)
        : ParallelEvaluatorSeverBase(_mpi_env, _world, _problem_defs), selection(_selection), crossover(_crossover), mutation(_mutation)
    {


    }

    ~ParallelEvaluatePopServerNonBlockingContinuousEvolution()
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_server_nonblock_ce_term.log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        // Send signal to slaves to indicate shutdown.
//        std::cout << "In destructor" << std::endl;
        std::string terminate = TERMINATE;
        std::vector<boost::mpi::request> term_msg_rqsts;
        for (int i = 1; i <= number_clients; ++i)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending terminate to " << i << std::endl;
            term_msg_rqsts.push_back(world.isend(i, max_tag, TERMINATE));
        }
        boost::mpi::wait_all(term_msg_rqsts.begin(), term_msg_rqsts.end());

        if (this->do_log > this->OFF)
            logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                             << ": Clearing running jobs" << std::endl;
        if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Clearing previously sent jobs";
        for(std::pair<const int, std::pair<boost::mpi::request, IndividualSPtr> > & job: jobs_running)
        {
            // Failed jobs.
            // set decision variables to something not so good.
            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " never received objective and constraints from worker for job " << job.first;
            boost::optional<boost::mpi::status> oss = job.second.first.test();
            if (!oss)
            {
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " sending of job to worker was unsuccessful." << std::endl;
                job.second.first.cancel();
            }
            else
            {
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " sending of job to worker was successful. But we never received a reply. Perhaps the generation incremented before the worker got to this job"
                                     << std::endl;
            }
        }
        jobs_running.clear();
        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }
    }

    void
    makeJobsApplyEAOperators(int num_jobs_made)
    {
        PopulationSPtr sub_pop = selection(num_jobs_made);
//        std::cout << "Sub pop for selection" << std::endl;
//        std::cout << sub_pop << std::endl;
        crossover(sub_pop);
        mutation(sub_pop);
        for(IndividualSPtr ind: *sub_pop)
        {
            jobs.push(ind);
        }
    }

    PopulationSPtr
    breedAndEvalPop(PopulationSPtr population)
    {
        return(breedAndEvalPop(population, NO_SAVE));
    }

    PopulationSPtr
    breedAndEvalPop(PopulationSPtr population, const boost::filesystem::path & save_dir)
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_server_nonblock_ce_breedeval" + std::to_string(++this->breed_and_eval_count) + ".log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Breed (and eval) population" << std::endl;

//        std::cout << "Broadcasting skeleton" << std::endl;
//        std::cout << "Real DVs: " << population->front()->numberOfRealDecisionVariables() << std::endl;
//        std::cout << "Int DVs: " << problem_defs->int_lowerbounds.size() << std::endl;
//        std::cout << "Objs: " << problem_defs->minimise_or_maximise.size() << std::endl;
//        std::cout << "Constraints: " << problem_defs->number_constraints << std::endl;

        generational_reproduction_size = population->size();
        selection.resetBreedingPop(population);
        PopulationSPtr offspring(new Population);
        std::string save_dir_s = save_dir.string();

        std::vector<boost::mpi::request> save_dir_msg_rqsts;
        for (int i = 1; i <= number_clients; ++i)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << i << " save directory: " << save_dir << std::endl;
            save_dir_msg_rqsts.push_back(world.isend(i, max_tag, save_dir_s));
        }

//        boost::mpi::broadcast(world, save_dir_s,0);

        if (this->do_log > this->OFF) logging_file << "Start of evolution and evaluation for generation " << gen_num++ << std::endl;

//        typedef std::pair<const int, boost::mpi::request> JobRunningT;
//        std::map<int, boost::mpi::request> jobs_running;

        for (int j = 0; j < CLIENT_QUEUE_SIZE; ++j)
        {
            for (int k = 0; k < number_clients; ++k)
            {
                if (jobs.size() == 0) makeJobsApplyEAOperators(2);
                decision_vars.first = jobs.front()->getRealDVVector();
                decision_vars.second = jobs.front()->getIntDVVector();
                int client_id = k + 1;
                if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << job_num << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
                jobs_running.insert(std::make_pair(job_num, std::make_pair(world.isend(client_id, job_num, dv_c), jobs.front())));
                jobs.pop();
                do{
                    ++job_num;
                    if (job_num > max_tag - 3 ) job_num = 0;
                }while (jobs_running.count(job_num) != 0);
            }
        }

        boost::mpi::wait_all(save_dir_msg_rqsts.begin(), save_dir_msg_rqsts.end());
        save_dir_msg_rqsts.clear();

//        mpi::wait_all(reqs_out.begin(), reqs_out.end());
        boost::mpi::status s;
        int results_received = 0;

        while (results_received < generational_reproduction_size)
        {
            //start a receive request, non-blocking
            s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            int client_id = s.source();
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << client_id << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
            IndividualSPtr ind = jobs_running[s.tag()].second;
            ind->setObjectives(objs_and_constraints.first);
            ind->setConstraints(objs_and_constraints.second);
            offspring->push_back(ind);
            if (add_offspring_2_mating_pool) selection.add2BreedingPop(ind);
            jobs_running.erase(s.tag());
            ++results_received;

            // Send out new job.
            if (jobs.size() == 0) makeJobsApplyEAOperators(2);
            decision_vars.first = jobs.front()->getRealDVVector();
            decision_vars.second = jobs.front()->getIntDVVector();


            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << job_num << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
            while (jobs_running.count(job_num) != 0) ++job_num;
            jobs_running.insert(std::make_pair(job_num, std::make_pair(world.isend(client_id, job_num, dv_c), jobs.front())));
            jobs.pop();
            do{
                ++job_num;
                if (job_num > max_tag - 3 ) job_num = 0;
            }while (jobs_running.count(job_num) != 0);
        }

//        for (int i = 0; i < number_clients; ++i)
//        {
//            int client_id = i + 1;
////            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
//            world.isend(client_id, max_tag, dv_c);
//        }

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }
        return(offspring);
    }

    void
    evalPop(PopulationSPtr population)
    {
        evalAndSavePop(population, NO_SAVE);
    }

    boost::optional<boost::mpi::status>
    waitForJobToFinish(std::ofstream & logging_file, int job_id = boost::mpi::any_tag)
    {
        //start a receive request, non-blocking
        boost::mpi::request r = world.irecv(boost::mpi::any_source, job_id, oc_c);
        //get start time
        std::time_t start_time, end_time;
        std::time(&start_time);
        int elapsed_time = 0;
        boost::optional<boost::mpi::status> osr;
        // test if we have made a receive.
        do
        {
            std::time(&end_time);
            elapsed_time = std::difftime(end_time, start_time);
            osr = r.test();

        }
        while(!osr && elapsed_time < this->timeout_time);

        //By now we either have received the data, or taken too long, so...
        if (!osr)
        {
            //we must have timed out
            if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Cancelling job " << job_id << " after waiting " << elapsed_time << " seconds." << std::endl;
            r.cancel();
            jobs_running.erase(job_id);
        }

        return osr;
    }

    void
    evalAndSavePop(PopulationSPtr population, const boost::filesystem::path & save_dir)
    {
        std::ofstream logging_file;
        if (this->do_log)
        {
            std::string file_name = "parallel_evaluate_server_nonblock_ce_evalsave" + std::to_string(++this->eval_and_save_count) + ".log";
            boost::filesystem::path log_file = this->log_directory / file_name;
            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
            if (!logging_file.is_open()) this->do_log = this->OFF;
        }

        if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Eval (and save) population" << std::endl;

        std::string save_dir_s = save_dir.string();
        std::vector<boost::mpi::request> save_dir_msg_rqsts;
        for (int i = 1; i <= number_clients; ++i)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << i << " save directory: " << save_dir << std::endl;
//            world.isend(i, max_tag - 1, save_dir.string());
            save_dir_msg_rqsts.push_back(world.isend(i, max_tag, save_dir_s));
        }

        if (this->do_log > this->OFF) logging_file << "Evaluating population with size: " << population->size() << std::endl;

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag -1))
        {
            if (this->do_log > this->OFF) logging_file << "problem: max tag too small, population too large for mpi" << std::endl;
        }

        int individual = 0;
//        std::vector<boost::mpi::request> reqs_out(number_clients);
        int num_initial_jobs = number_clients;
        if (num_initial_jobs > population->populationSize()) num_initial_jobs = population->populationSize();
        for (; individual < num_initial_jobs; ++individual)
        {
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = individual + 1;
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
            if (jobs_running.count(individual) != 0) waitForJobToFinish(logging_file, individual);
            jobs_running.insert(std::make_pair(individual, std::make_pair(world.isend(client_id, individual, dv_c), (*population)[individual])));
        }

//        // keep jobs out there for slave eficiency (so queue an additional job for each slave)
//        int num_additional_jobs = number_clients;
//        if ((num_additional_jobs + num_initial_jobs) > population->populationSize()) num_additional_jobs = population->populationSize() - num_initial_jobs;
//        for (int i = 0; i < num_additional_jobs; ++i)
//        {
//            decision_vars.first = (*population)[individual]->getRealDVVector();
//            decision_vars.second = (*population)[individual]->getIntDVVector();
//            int client_id = individual + 1 - num_initial_jobs;
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
//            if (jobs_running.count(individual) != 0)
//            {
//                //start a receive request, non-blocking
//                boost::mpi::request r = world.irecv(boost::mpi::any_source, individual, oc_c);
//                //get start time
//                std::time_t start_time = std::time(NULL);
//                // test if we have made a receive.
//                boost::optional<boost::mpi::status> osr = r.test();
//                if (!osr)
//                {
//                    //loop until we have received, or taken too long
//                    while (!osr && (std::difftime(std::time(NULL),start_time) < this->timeout_time))
//                    {
//                        //wait a bit.
//                        osr = r.test();
//                    }
//                }
//
//                //By now we either have received the data, or taken too long, so...
//                if (!osr)
//                {
//                    //we must have timed out
//                    r.cancel();
//                }
//                jobs_running.erase(individual);
//
//            }
//            jobs_running.insert(std::make_pair(individual, std::make_pair(world.isend(client_id, individual, dv_c), (*population)[individual])));
//            ++individual;
//        }

//        boost::mpi::wait_all(save_dir_msg_rqsts.begin(), save_dir_msg_rqsts.end());
//        save_dir_msg_rqsts.clear();

//        mpi::wait_all(reqs_out.begin(), reqs_out.end());
//        boost::mpi::status s;

        while (individual < population->populationSize())
        {
            //start a receive request, non-blocking
            boost::mpi::request r = world.irecv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
            boost::mpi::status s = r.wait();
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
            jobs_running.erase(s.tag());
//            //get start time
//            std::time_t start_time = std::time(NULL);
//            // test if we have made a receive.
//            boost::optional<boost::mpi::status> osr = r.test();
//            if (!osr)
//            {
//                //loop until we have received, or taken too long
//                while (!osr && (std::difftime(std::time(NULL),start_time) < this->timeout_time))
//                {
//                    //wait a bit.
//                    osr = r.test();
//                }
//            }
//
//            //By now we either have received the data, or taken too long, so...
//            if (!osr)
//            {
//                //we must have timed out
//                r.cancel();
//            }
//            else
//            {

            // Send out new job.
            decision_vars.first = (*population)[individual]->getRealDVVector();
            decision_vars.second = (*population)[individual]->getIntDVVector();
            int client_id = s.source();
            if (jobs_running.count(individual) != 0) waitForJobToFinish(logging_file, individual);
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVS: " << decision_vars << std::endl;
            jobs_running.insert(std::make_pair(individual, std::make_pair(world.isend(client_id, individual, dv_c), (*population)[individual])));

                //Process received job
                if (s.tag() < population->size())
                {
                    (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
                    (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
                }

//                // send out job for next individual. Make sure there isn't a job already sent out but not yet received with job id equal to the individual.
//                if (jobs_running.count(individual) != 0)
//                {
//                    //start a receive request, non-blocking
//                    boost::mpi::request r = world.irecv(boost::mpi::any_source, individual, oc_c);
//                    //get start time
//                    std::time_t start_time = std::time(NULL);
//                    // test if we have made a receive.
//                    boost::optional<boost::mpi::status> osr = r.test();
//                    if (!osr)
//                    {
//                        //loop until we have received, or taken too long
//                        while (!osr && (std::difftime(std::time(NULL),start_time) < this->timeout_time))
//                        {
//                            //wait a bit.
//                            osr = r.test();
//                        }
//                    }
//
//                    //By now we either have received the data, or taken too long, so...
//                    if (!osr)
//                    {
//                        //we must have timed out
//                        r.cancel();
//                    }
//                    jobs_running.erase(individual);
//
//                }

                ++individual;
        }

        //Try and get a receive from remaining jobs still running
        for (int j = 0; j < jobs_running.size(); ++j)
        {
            //start a receive request, non-blocking
            boost::optional<boost::mpi::status> os = waitForJobToFinish(logging_file);
            if (os)
            {
                boost::mpi::status s = os.get();
                if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
                if (s.tag() < population->size())
                {
                    (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
                    (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
                }
                jobs_running.erase(s.tag());
            }
        }

        // Cancel jobs running which we have not received a response from (timed out)
        for(std::pair<const int, std::pair<boost::mpi::request, IndividualSPtr> > & job: jobs_running)
        {
            // Failed jobs.
            // set decision variables to something not so good.
            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " never received objective and constraints from worker for individual " << job.first;
            boost::optional<boost::mpi::status> oss = job.second.first.test();
            if (!oss)
            {
                if (this->do_log > this->OFF)
                    logging_file
                                     << ", because sending of individual to worker was unsuccessful." << std::endl;
                job.second.first.cancel();
            }
            else
            {
                if (this->do_log > this->OFF)
                    logging_file
                                     << ". While sending of individual to worker was successful, we never received a reply. Perhaps the generation incremented before a worker got to this job"
                                     << std::endl;
            }
            (*population)[job.first]->setObjectives(this->worst_objs_and_constraints.first);
            (*population)[job.first]->setConstraints(this->worst_objs_and_constraints.second);
        }

//        for (int i = 0; i < number_clients; ++i)
//        {
//            int client_id = i + 1;
////            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
//            world.isend(client_id, max_tag, dv_c);
//        }
        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }

    }



    void
    operator()(PopulationSPtr population)
    {
        evalAndSavePop(population, NO_SAVE);
    }

    void
    operator()(PopulationSPtr population, const boost::filesystem::path & save_dir)
    {
        evalAndSavePop(population, save_dir);
    }
};



class ParallelEvaluatePopClientNonBlockingContinuousEvolution : public ParallelEvaluatorClientBase
{
    ObjectivesAndConstraintsBase & eval;
    std::queue< std::tuple< int, int, std::pair<std::vector<double>, std::vector<int> > > > jobs; //First is gen_number, second is job number, third is decision variable
    int gen_number;

public:
    ParallelEvaluatePopClientNonBlockingContinuousEvolution(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
        : ParallelEvaluatorClientBase(_mpi_env, _world, _problem_defs), eval(_eval), gen_number(0)
    {

    }

    void
    operator()()
    {

        std::ofstream logging_file;

        bool do_continue = true;
        bool in_generation = true;
        boost::mpi::request rq;
        bool first_time = true;
        std::string save_dir_s;
        boost::filesystem::path save_dir;
        bool do_save;

        while (do_continue)
        {
            if (this->do_log)
            {
                if (logging_file.is_open()) logging_file.close();
                std::string file_name = "parallel_evaluate_client" + std::to_string(world.rank()) + "_nonblock_ce_gen" + std::to_string(++this->gen_num) + std::string(".log");
                boost::filesystem::path log_file = this->log_directory / file_name;
                logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
                if (!logging_file.is_open()) this->do_log = this->OFF;
            }

            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " Waiting to receive start-of-generation message (e.g. location of save-dir)" << std::endl;
            boost::mpi::request r = world.irecv(0, max_tag, save_dir_s);
            boost::mpi::status s = r.wait();
            if (save_dir_s == NO_SAVE)
            {
                do_save = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Not saving evaluation to file" << std::endl;
                in_generation = true;
                ++gen_number;
            }
            else if (save_dir_s == TERMINATE)
            {
                in_generation = false;
                do_continue = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Terminating" << std::endl;
                break;
            }
            else
            {
                do_save = true;
                save_dir = save_dir_s;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Saving evaluation here: " << save_dir_s << std::endl;
                in_generation = true;
                ++gen_number;
            }
            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " Start of generation " << gen_number << std::endl;

            bool print_waiting = true;
            while (in_generation)
            {
                if (this->do_log > this->OFF && print_waiting)
                {
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Checking if messages to receive" << std::endl;
                    print_waiting = false;
                }

                bool jobs_2_receive = true;
                while (jobs_2_receive)
                {
//                    boost::optional<boost::mpi::status> s_end_of_gen = world.iprobe(0, max_tag);
//                    if (s_end_of_gen)
//                    {
//                        // Then next generation has commenced.
//                        in_generation = false;
//                        break;
//                    }
                    boost::optional<boost::mpi::status> s_is_job = world.iprobe(0, boost::mpi::any_tag);
                    if (s_is_job)
                    {
                        if (s_is_job.get().tag() != max_tag)
                        {
                            boost::mpi::status s_job = world.recv(0, boost::mpi::any_tag, dv_c);
                            print_waiting = true;
                            if (this->do_log > this->OFF)
                                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                                 << " received " << decision_vars << " for individual " << s_job.tag() << std::endl;
                            jobs.push(std::make_tuple(gen_number, s_job.tag(), decision_vars));
                        }
                        else // I.e. there is a message indicating next generation has commenced....
                        {
                            // Then next generation has commenced.
                            in_generation = false;
                            break;
                        }

                    }
                    else
                    {
                        jobs_2_receive = false;
                    }
                }


                if (in_generation)
                {
                    if (!jobs.empty())
                    {
                        while (std::get<0>(jobs.front()) < std::get<0>(jobs.back()))
                        {
                            jobs.pop();
                        }

                        // Now run the job at the front of the queue
                        std::tuple< int, int, std::pair<std::vector<double>, std::vector<int> > > & job = jobs.front();
                            //calc objective
                            if (!do_save)
                            {
                                objs_and_constraints = eval(std::get<2>(job).first, std::get<2>(job).second);
                            }
                            else
                            {
                                boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(std::get<1>(job)));
                                if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
                                objs_and_constraints = eval(std::get<2>(job).first, std::get<2>(job).second, save_ind_dir);
                            }

                            if (this->do_log > this->OFF)
                                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                                 << " sending " << objs_and_constraints << " for individual " << std::get<1>(job);
                            logging_file.flush();
//                                         << std::endl;

                            // Wait for previous job results to be successfully sent.
                            if (!first_time)
                            {
                                rq.wait();
                            }
                            else
                            {
                                first_time = false;
                            }
                            rq = world.isend(0, std::get<1>(job), oc_c);
                            if (this->do_log > this->OFF)
                                logging_file << "; now sent"  << std::endl;
                        jobs.pop();
                    }

                }
            }

        }

    }
};



#endif /* ParallelEvaluator_h */
