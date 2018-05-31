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
#include "serialize-tuple-master/serialize_tuple.h"
#include <boost/date_time.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <queue>
#include <set>
#include <unordered_map>
#include "Selection.hpp"
#include "Crossover.hpp"
#include "Mutation.hpp"


namespace{

inline std::ostream& operator << (std::ostream& os, const std::tuple<int, std::vector<double>, std::vector<int> >& v)
{
    os << "[";
    for (std::vector<double>::const_iterator ii = std::get<1>(v).begin(); ii != std::get<1>(v).end(); ++ii)
    {
        os << " " << *ii;
    }
    os << "; ";
    for (std::vector<int>::const_iterator ii = std::get<2>(v).begin(); ii != std::get<2>(v).end(); ++ii)
    {
        os << " " << *ii;
    }
    os << " ] : Gen " << std::get<0>(v);
    return os;
}

inline std::ostream& operator << (std::ostream& os, const std::tuple<std::vector<double>, std::vector<double>, int, std::vector<double>, std::vector<int> >& v)
{
    os << "(";
    for (std::vector<double>::const_iterator ii = std::get<0>(v).begin(); ii != std::get<0>(v).end(); ++ii)
    {
        os << " " << *ii;
    }
    os << "; ";
    for (std::vector<double>::const_iterator ii = std::get<1>(v).begin(); ii != std::get<1>(v).end(); ++ii)
    {
        os << " " << *ii;
    }
    os << " ) <- ";

    os << "[";
    for (std::vector<double>::const_iterator ii = std::get<3>(v).begin(); ii != std::get<3>(v).end(); ++ii)
    {
        os << " " << *ii;
    }
    os << "; ";
    for (std::vector<int>::const_iterator ii = std::get<4>(v).begin(); ii != std::get<4>(v).end(); ++ii)
    {
        os << " " << *ii;
    }
    os << " ] : Gen " << std::get<2>(v);
    return os;
}




}

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
//    std::pair<std::vector<double>, std::vector<int> > decision_vars;
//    std::tuple<std::vector<double>, std::vector<int>, std::vector<int> > decision_vars2;
//    std::pair<std::vector<double>, std::vector<double> > objs_and_constraints;
    std::tuple<std::vector<double>, std::vector<double>, int, std::vector<double>, std::vector<int> > objs_constraints_gen_num_dvs;
    std::tuple<int, std::vector<double>, std::vector<int> > gen_number_dvs;
    std::pair<std::vector<double>, std::vector<double> > worst_objs_and_constraints;
    std::pair<std::string, int> save_dir_gen_nmbr;
//    boost::mpi::content dv_c;
//    boost::mpi::content oc_c;
//    boost::mpi::content ocgndv_c;
//    boost::mpi::content gndv_c;
    int max_tag;
    int do_log;
    int gen_num = 0;
    boost::filesystem::path log_directory;
    bool delete_previous_logfile = true;
    boost::filesystem::path previous_log_file = "";
    boost::filesystem::path log_file = "";
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

    void
    makeLogFile(std::ofstream &logging_file, std::string & base_log_file_name, int _gen_count, std::string subdir_name)
    {
        std::string file_name = base_log_file_name;
        if (_gen_count >= 0) file_name = file_name + "_" + std::to_string(
                _gen_count);
        file_name = file_name + "_" + boost::asio::ip::host_name() + "_Rank" + std::to_string(world.rank()) + ".log";

        log_file = log_directory / subdir_name;
        if (!(exists(log_file)))
        {
            try
            {
                create_directories(log_file);
//                    std::cout << "path " << path.first << " did not exist, so created\n";
            }
            catch(boost::filesystem::filesystem_error& e)
            {
                std::cout << "Attempted to create " << log_file.string().c_str() << " but was unable\n";
                std::cout << e.what() << "\n";
//                    this->do_log = false;
                log_file = log_directory;
            }
        }

        log_file = log_file / file_name;
        logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::app);
        if (!logging_file.is_open()) do_log = OFF;
    }

    void
    deletePreviousLog(std::string & base_log_file_name, int _gen_count, std::string subdir_name)
    {
        std::string file_name = base_log_file_name;
        if (_gen_count >= 0) file_name = file_name + "_" + std::to_string(
                _gen_count);
        file_name = file_name + "_" + boost::asio::ip::host_name() + ".log";

        previous_log_file = log_directory / subdir_name /file_name ;
        if (this->delete_previous_logfile && !this->previous_log_file.empty())
        {
            if (exists(previous_log_file)) boost::filesystem::remove_all(previous_log_file);
        }
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
//        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
//        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_constraints_gen_num_dvs),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(gen_number_dvs),0);

//        std::cout << "Getting content for skeleton" << std::endl;
//        dv_c = boost::mpi::get_content(decision_vars);
//        oc_c = boost::mpi::get_content(objs_and_constraints);
//        ocgndv_c = boost::mpi::get_content(objs_constraints_gen_num_dvs);
//        gndv_c  = boost::mpi::get_content(gen_number_dvs);
//        std::cout << "Completed initialisation Client" << std::endl;
    }

    virtual void operator()()=0;

};

class ParallelEvaluatorSeverBase : public ParallelEvaluatorBase, public EvaluatePopulationBase
{
private:
    typedef std::tuple<int, std::vector<double>, std::vector<int> > JobsSentInfo; //first is gen number, then DVs.
    typedef std::list<std::tuple<boost::mpi::request, int,  JobsSentInfo> > JobsSentList; //First is mpi_request, 2nd is job number, 3rd is tuple of objs, constraints, gen_num and decision vars
    JobsSentList jobs_sending;
    const int MAX_JOB_NUM = std::numeric_limits<int>::max()-2;

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

//    std::get<0>(decision_vars

//        decision_vars = std::pair<std::vector<double>, std::vector<int> >
//            (   std::piecewise_construct,
//                std::forward_as_tuple(std::vector<double>(problem_defs->real_lowerbounds.size(), 0.0)),
//                std::forward_as_tuple(std::vector<int>(problem_defs->int_lowerbounds.size(), 0))
//            );
//
//        objs_and_constraints = std::pair<std::vector<double>, std::vector<double> >
//            (   std::piecewise_construct,
//                std::forward_as_tuple(std::vector<double>(problem_defs->minimise_or_maximise.size(), 0.0)),
//                std::forward_as_tuple(std::vector<double>(problem_defs->number_constraints, 0.0))
//            );

        std::get<0>(objs_constraints_gen_num_dvs) = std::vector<double>(problem_defs->minimise_or_maximise.size(), 0.0);
        std::get<1>(objs_constraints_gen_num_dvs) = std::vector<double>(problem_defs->number_constraints, 0.0);
        std::get<2>(objs_constraints_gen_num_dvs) = 0;
        std::get<3>(objs_constraints_gen_num_dvs) = std::vector<double>(problem_defs->real_lowerbounds.size(), 0.0);
        std::get<4>(objs_constraints_gen_num_dvs) = std::vector<int>(problem_defs->int_lowerbounds.size(), 0);

        std::get<0>(gen_number_dvs) = 0;
        std::get<1>(gen_number_dvs) = std::vector<double>(problem_defs->real_lowerbounds.size(), 0.0);
        std::get<2>(gen_number_dvs) = std::vector<int>(problem_defs->int_lowerbounds.size(), 0);


//        std::cout << "Broadcasting skeleton" << std::endl;
//        boost::mpi::broadcast(world, boost::mpi::skeleton(decision_vars),0);
//        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_and_constraints),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(objs_constraints_gen_num_dvs),0);
        boost::mpi::broadcast(world, boost::mpi::skeleton(gen_number_dvs),0);

//        std::cout << "Getting content for skeleton" << std::endl;
//        dv_c = boost::mpi::get_content(decision_vars);
//        oc_c = boost::mpi::get_content(objs_and_constraints);
//        ocgndv_c = boost::mpi::get_content(objs_constraints_gen_num_dvs);
//        gndv_c  = boost::mpi::get_content(gen_number_dvs);

//        std::cout << "Completed initialisation Server" << std::endl;
    }

    void
    terminate()
    {
        std::string base_log_file_name = "ParallelEvaluatePopServer_Termination";
        std::string log_subdir = "ParallelEvaluateServerLogs";
        std::ofstream logging_file;
        if (this->do_log)
        {
            makeLogFile(logging_file, base_log_file_name, -1, log_subdir);
        }
        // Send signal to slaves to indicate shutdown.
//        std::cout << "In destructor" << std::endl;

        std::vector<boost::mpi::request> term_msg_rqsts;
        for (int i = 1; i <= number_clients; ++i)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending terminate to " << i << std::endl;
            term_msg_rqsts.push_back(world.isend(i, max_tag, TERMINATE));
        }
        boost::mpi::wait_all(term_msg_rqsts.begin(), term_msg_rqsts.end());

        deleteAllJobs(logging_file);

        if (this->do_log > this->OFF)
        {
            if (logging_file.is_open()) logging_file.close();
        }
    }



    void
    sendJob(int _gen_num, const std::vector<double> & real_dvs, const std::vector<int>& int_dvs, int & job_num, int client_id, std::ofstream & logging_file)
    {
        job_num += 1;

        std::get<0>(gen_number_dvs) = _gen_num;
        std::get<1>(gen_number_dvs) = real_dvs;
        std::get<2>(gen_number_dvs) = int_dvs;

        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << job_num << " with " << real_dvs.size() << ", " << int_dvs.size() << " DVS: " << gen_number_dvs << std::endl;
        boost::mpi::request rq;
        jobs_sending.push_front(std::make_tuple(rq, job_num, gen_number_dvs));
        std::tuple<boost::mpi::request, int,  JobsSentInfo> & job = jobs_sending.front();
            std::get<0>(job) = world.isend(client_id, std::get<1>(job), boost::mpi::get_content(std::get<2>(job)));

        if (job_num > max_tag - 3) job_num = 0;

    }

    void
    checkJobsSent(std::ofstream & logging_file)
    {
        // Test sends back, and remove those completed.
        boost::optional<boost::mpi::status> os;
        do{
            os = boost::optional<boost::mpi::status>();
            if (!jobs_sending.empty())
            {
                std::tuple<boost::mpi::request, int,  JobsSentInfo> & job_sending = jobs_sending.back();
                os = std::get<0>(job_sending).test();
                if (os)
                {
                    if (this->do_log > this->OFF)
                        logging_file <<  world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " individual/job number " << std::get<1>(job_sending) << " now sent" << std::endl;
                    jobs_sending.pop_back();
                }
            }
        } while (os);
    }

    void
    deleteOldJobs(std::ofstream & logging_file, int old_gen_number)
    {
        // Test sends back, and remove those completed.
        boost::optional<boost::mpi::status> os;
        bool do_continue = false;
        do{
            do_continue = false;
            if (!jobs_sending.empty())
            {
                std::tuple<boost::mpi::request, int,  JobsSentInfo> & job_sending = jobs_sending.back();
                os = std::get<0>(job_sending).test();
                if (os)
                {
                    if (this->do_log > this->OFF)
                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " individual/job number " << std::get<1>(job_sending) << "now sent" << std::endl;
                    jobs_sending.pop_back();
                    do_continue = true;
                }
                else
                {
                    if (std::get<0>(std::get<2>(job_sending)) < old_gen_number)
                    {
                        if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Cancelling job " << std::get<1>(job_sending) << " from generation " << std::get<0>(std::get<2>(job_sending)) << ". Job is too old." << std::endl;
                        std::get<0>(job_sending).cancel();
                        jobs_sending.pop_back();
                        do_continue = true;
                    }
                }
            }
        } while (do_continue);
    }

    void
    deleteAllJobs(std::ofstream & logging_file)
    {
        if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Clearing previously sent jobs" << std::endl;
        for(std::tuple<boost::mpi::request, int,  JobsSentInfo> & job: jobs_sending)
        {
            // Failed jobs.
            boost::optional<boost::mpi::status> oss = std::get<0>(job).test();
            if (!oss)
            {
                std::get<0>(job).cancel();
            }
            else
            {
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " individual/job number " << std::get<1>(job) << "now sent" << std::endl;
            }
        }
        jobs_sending.clear();
    }

    boost::optional<boost::mpi::status>
    waitForJobToFinish(std::ofstream & logging_file, int job_id = boost::mpi::any_tag)
    {
        //start a receive request, non-blocking
        boost::mpi::request r = world.irecv(boost::mpi::any_source, job_id, boost::mpi::get_content(objs_constraints_gen_num_dvs));
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
        }

        return osr;
    }

    void
    evalAndSavePopImpl(PopulationSPtr population, const boost::filesystem::path & save_dir, int _gen_num)
    {
        std::ofstream logging_file;
        std::string base_log_file_name = "ParallelEvaluatePopServer_EvalSave";
        std::string log_subdir = "ParallelEvaluateServerLogs";
        if (this->do_log)
        {
            makeLogFile(logging_file, base_log_file_name, _gen_num, log_subdir);
        }


        if (this->do_log > this->OFF) logging_file << world.rank() << " " <<  boost::posix_time::second_clock::local_time() << ": Eval (and save) population" << std::endl;

        save_dir_gen_nmbr.first = save_dir.string();
        save_dir_gen_nmbr.second = _gen_num;

        std::vector<boost::mpi::request> save_dir_msg_rqsts;
        for (int i = 1; i <= number_clients; ++i)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << i << " save directory: " << save_dir << " and generational number: " << _gen_num << std::endl;
            save_dir_msg_rqsts.push_back(world.isend(i, max_tag, save_dir_gen_nmbr));
        }

        if (this->do_log > this->OFF) logging_file << "First defaulting obj and constraints of all population members to worse case values" << std::endl;
        for (IndividualSPtr ind: *population)
        {
            ind->setObjectives(this->worst_objs_and_constraints.first);
            ind->setConstraints(this->worst_objs_and_constraints.second);
        }

        if (this->do_log > this->OFF) logging_file << "Evaluating population with size: " << population->size() << std::endl;

        //Sanity check - that we can represent each individual by an mpi tag.
        if (population->populationSize() > (max_tag -1))
        {
            if (this->do_log > this->OFF) logging_file << "problem: max tag too small, population too large for mpi" << std::endl;
        }

        deleteAllJobs(logging_file);

        int individual = 0;
//        std::vector<boost::mpi::request> reqs_out(number_clients);
        int num_initial_jobs = number_clients;
        if (num_initial_jobs > population->populationSize()) num_initial_jobs = population->populationSize();
        for (; individual < num_initial_jobs; )
        {
            // sendJob increments 'indivudal'
            int client_id = individual + 1;
            sendJob(_gen_num, (*population)[individual]->getRealDVVector(), (*population)[individual]->getIntDVVector(), individual, client_id, logging_file);
        }

        //Check messages have all been sent
        boost::mpi::wait_all(save_dir_msg_rqsts.begin(), save_dir_msg_rqsts.end());
        save_dir_msg_rqsts.clear();

        checkJobsSent(logging_file);
        int results_received = 0;

        while (individual < population->populationSize())
        {
            //start a receive request, non-blocking
            boost::mpi::request r_results = world.irecv(boost::mpi::any_source, boost::mpi::any_tag, boost::mpi::get_content(objs_constraints_gen_num_dvs));
            boost::mpi::status s_results = r_results.wait();
            int client_id = s_results.source();
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << client_id << " individual/job number " << s_results.tag() << " with " << objs_constraints_gen_num_dvs << std::endl;

            // Send out new job.
            sendJob(_gen_num, (*population)[individual]->getRealDVVector(), (*population)[individual]->getIntDVVector(), individual, client_id, logging_file);

            //Process received job
            if (s_results.tag() < population->size())
            {
                if ( ((*population)[s_results.tag() - 1]->getRealDVVector() == std::get<3>(objs_constraints_gen_num_dvs))
                    &&  ((*population)[s_results.tag() - 1]->getIntDVVector() == std::get<4>(objs_constraints_gen_num_dvs)) )
                // we -1 in pop index as vectors are indexed from 0, while the tag is the ind. number indexed from 1.
                {
                    (*population)[s_results.tag()]->setObjectives(std::get<0>(objs_constraints_gen_num_dvs));
                    (*population)[s_results.tag()]->setConstraints(std::get<1>(objs_constraints_gen_num_dvs));
                    results_received += 1;
                }
            }
            // sendJob increments 'indivudal'
//            ++individual;
            checkJobsSent(logging_file);
        }

        //Try and get a receive from remaining jobs still running
        boost::optional<boost::mpi::status> os;
        do
        {
            if (results_received >= population->size()) break;
            os = waitForJobToFinish(logging_file);
            if (os)
            {
                boost::mpi::status s_results = os.get();
                if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s_results.source() << " individual/job number " << s_results.tag() << " with " << objs_constraints_gen_num_dvs << std::endl;

                //Process received job
                if (s_results.tag() <= population->size())
                {
                    if ( ((*population)[s_results.tag() - 1]->getRealDVVector() == std::get<3>(objs_constraints_gen_num_dvs))
                        &&  ((*population)[s_results.tag() - 1]->getIntDVVector() == std::get<4>(objs_constraints_gen_num_dvs)) )
                        // we -1 in pop index as vectors are indexed from 0, while the tag is the ind. number indexed from 1.
                    {
                        (*population)[s_results.tag()]->setObjectives(std::get<0>(objs_constraints_gen_num_dvs));
                        (*population)[s_results.tag()]->setConstraints(std::get<1>(objs_constraints_gen_num_dvs));
                        results_received += 1;
                    }
                }
            }
        } while (os);

        for (int j = 0; j < results_received; ++j)
        {
            //start a receive request, non-blocking

        }

        deleteAllJobs(logging_file);

        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }

        if (this->do_log)
        {
            if (_gen_num>3) deletePreviousLog(base_log_file_name, _gen_num-3, log_subdir);
        }

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
        terminate();
    }

    void
    evalPop(PopulationSPtr population, boost::filesystem::path save_dir)
    {
        ++gen_num;
        evalAndSavePopImpl(population, save_dir, gen_num);
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



template <typename RNG_PE = std::mt19937>
class ParallelEvaluatePopServerNonBlockingContinuousEvolution : public ParallelEvaluatorSeverBase
{

private:
    std::queue<IndividualSPtr> jobs;
//    std::unordered_map<int, std::pair<boost::mpi::request, IndividualSPtr> > jobs_running;
    const int CLIENT_QUEUE_SIZE = 2;
//    const bool addOffspring2Selection = true;
    bool add_offspring_2_mating_pool = true;
    int generational_reproduction_size;
    TournamentSelectionContinuousEvolution<RNG_PE> & selection;
    CombinedRealIntCrossover<RNG_PE> & crossover;
    CombinedRealIntMutation<RNG_PE> & mutation;
    int job_num = 0;
//    int gen_num = 0;
//    int breed_and_eval_count = 0;
//    int eval_and_save_count = 0;

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
        terminate();
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
        std::string base_log_file_name = "ParallelEvaluatePopServer_BreedEval";
        std::string log_subdir = "ParallelEvaluateServerLogs";
        std::ofstream logging_file;
//        breed_and_eval_count += 1;
        gen_num += 1;
        if (this->do_log)
        {
            makeLogFile(logging_file, base_log_file_name, gen_num, log_subdir);
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

        save_dir_gen_nmbr.first = save_dir.string();
        save_dir_gen_nmbr.second = gen_num;

        std::vector<boost::mpi::request> save_dir_msg_rqsts;
        for (int i = 1; i <= number_clients; ++i)
        {
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << i << " save directory: " << save_dir << " and generational number: " << gen_num << std::endl;
            save_dir_msg_rqsts.push_back(world.isend(i, max_tag, save_dir_gen_nmbr));
        }

//        boost::mpi::broadcast(world, save_dir_s,0);


        if (this->do_log > this->OFF) logging_file << "Start of evolution and evaluation for generation " << gen_num << std::endl;

//        typedef std::pair<const int, boost::mpi::request> JobRunningT;
//        std::map<int, boost::mpi::request> jobs_running;

        for (int j = 0; j < CLIENT_QUEUE_SIZE; ++j)
        {
            for (int k = 0; k < number_clients; ++k)
            {
                job_num += 1;
                if (jobs.size() == 0) makeJobsApplyEAOperators(2);
                int client_id = k + 1;
                sendJob(gen_num, jobs.front()->getRealDVVector(), jobs.front()->getIntDVVector(), job_num, client_id, logging_file);
                jobs.pop();
            }
        }

        boost::mpi::wait_all(save_dir_msg_rqsts.begin(), save_dir_msg_rqsts.end());
        save_dir_msg_rqsts.clear();

        //Check jobs sent.
        checkJobsSent(logging_file);


//        mpi::wait_all(reqs_out.begin(), reqs_out.end());
        boost::mpi::status s;
        int results_received = 0;

        while (results_received < generational_reproduction_size)
        {
            //start a receive request, non-blocking

            boost::mpi::request r_results = world.irecv(boost::mpi::any_source, boost::mpi::any_tag, boost::mpi::get_content(objs_constraints_gen_num_dvs));
            boost::mpi::status s_results = r_results.wait();
            int client_id = s_results.source();
            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << client_id << " individual/job number " << s.tag() << " with " << objs_constraints_gen_num_dvs << std::endl;

            // Send out new job.
            if (jobs.size() == 0) makeJobsApplyEAOperators(2);
            sendJob(gen_num, jobs.front()->getRealDVVector(), jobs.front()->getIntDVVector(), job_num, client_id, logging_file);
            jobs.pop();


            //process results
            IndividualSPtr ind(new Individual(this->problem_defs));
            ind->setRealDVs(std::get<3>(objs_constraints_gen_num_dvs));
            ind->setIntDVs(std::get<4>(objs_constraints_gen_num_dvs));
            ind->setObjectives(std::get<0>(objs_constraints_gen_num_dvs));
            ind->setConstraints(std::get<1>(objs_constraints_gen_num_dvs));
            offspring->push_back(ind);
            if (add_offspring_2_mating_pool) selection.add2BreedingPop(ind);
            ++results_received;

            //Check jobs sent.
            checkJobsSent(logging_file);
        }

        deleteOldJobs(logging_file, gen_num - 2);


        if (this->do_log)
        {
            if (logging_file.is_open()) logging_file.close();
        }

        if (this->do_log)
        {
            if (gen_num>3) deletePreviousLog(base_log_file_name, gen_num-3, log_subdir);
        }

        return(offspring);
    }


    void
    evalPop(PopulationSPtr population)
    {
        evalAndSavePop(population, NO_SAVE);
    }



    void
    evalAndSavePop(PopulationSPtr population, const boost::filesystem::path & save_dir)
    {
        evalAndSavePopImpl(population, save_dir, -1);
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



class ParallelEvaluatePopClientNonBlocking : public ParallelEvaluatorClientBase
{
    ObjectivesAndConstraintsBase & eval;
//    std::queue< std::tuple< int, int, std::pair<std::vector<double>, std::vector<int> > > > jobs;

    struct Jobs2DoCompare {
        typedef std::pair< int, std::tuple<int, std::vector<double>, std::vector<int> > > JobInfo;

        bool operator() (const JobInfo& x, const JobInfo& y) const
        {
            return (std::get<0>(x.second)) > (std::get<0>(y.second));
        }
        typedef std::tuple< int, int, std::tuple<int, std::vector<double>, std::vector<int> > > first_argument_type;
        typedef std::tuple< int, int, std::tuple<int, std::vector<double>, std::vector<int> > > second_argument_type;
        typedef bool result_type;
    };

    Jobs2DoCompare jobs_sort;
    typedef std::pair< int, std::tuple<int, std::vector<double>, std::vector<int> > > Job2DoInfo; //First is job number, third tuple of gen number then  decision variables
    typedef std::multiset< Job2DoInfo, Jobs2DoCompare > Jobs2DoMultiSet;
    Jobs2DoMultiSet jobs_2_do;
    typedef std::tuple<std::vector<double>, std::vector<double>, int, std::vector<double>, std::vector<int> > JobsDoneInfo;
    typedef std::list<std::tuple<boost::mpi::request, int,  JobsDoneInfo> > JobsDoneList; //First is mpi_request, 2nd is job number, 3rd is tuple of objs, constraints, gen_num and decision vars
    JobsDoneList jobs_done;
    const int MAX_JOBS_ON_QUEUE = 20;
//    int gen_number;

public:
    ParallelEvaluatePopClientNonBlocking(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
        : ParallelEvaluatorClientBase(_mpi_env, _world, _problem_defs), eval(_eval), jobs_2_do(jobs_sort)
    {

    }

    void
    operator()()
    {

        std::ofstream logging_file;
        std::string log_file_base_name = "ParallelEvaluatePopClientNonBlocking";
        std::string log_subdir_name = "ParallelEvaluateClientLogs";

        bool do_continue = true;
        bool in_generation = true;
//        boost::mpi::request rq;
        bool first_time = true;
        std::string save_dir_s;
        boost::filesystem::path save_dir;
        bool do_save;
        std::pair<std::vector<double>, std::vector<double> > objs_and_constraints;

        while (do_continue)
        {
            // Start of new generation - get generation number and save directory if we are to save the results of the evaluations.
            std::stringstream logging_file_tempry;
            if (do_log > OFF)
                logging_file_tempry << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                    << " Waiting to receive start-of-generation message (e.g. location of save-dir)" << std::endl;
            boost::mpi::request r = world.irecv(0, max_tag, save_dir_gen_nmbr);
            boost::mpi::status s = r.wait();
            save_dir_s = save_dir_gen_nmbr.first;
            gen_num = save_dir_gen_nmbr.second;

            // Are there any other save dir gen number messages - we want to get the latest....
            boost::optional<boost::mpi::status> so = world.iprobe(0, max_tag);
            while (so)
            {
                boost::mpi::request r = world.irecv(0, max_tag, save_dir_gen_nmbr);
                boost::mpi::status s = r.wait();
                if (save_dir_gen_nmbr.second > gen_num)
                {
                    save_dir_s = save_dir_gen_nmbr.first;
                    gen_num = save_dir_gen_nmbr.second;
                }
                so = world.iprobe(0, max_tag);
            }

            // Set up logging file
            if (do_log > OFF)
            {

                if (logging_file.is_open()) logging_file.close();
                makeLogFile(logging_file, log_file_base_name, gen_num, log_subdir_name);
                if (do_log > OFF) logging_file << logging_file_tempry.str().c_str();
            }


            // Set up saving directory, if we were given one.
            if (save_dir_s == NO_SAVE)
            {
                do_save = false;
                if (this->do_log > this->OFF)
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Not saving evaluation to file" << std::endl;
                in_generation = true;
//                ++gen_number;
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
//                ++gen_number;
            }
            if (this->do_log > this->OFF)
                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                 << " Start of generation " << gen_num << std::endl;

            bool print_waiting = true;
            while (in_generation)
            {
                if (this->do_log > this->OFF && print_waiting)
                {
                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                     << " Checking if messages to receive" << std::endl;
                    print_waiting = false;
                }

                bool is_jobs_2_receive = true;
                while (is_jobs_2_receive)
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
                            boost::mpi::request r_job = world.irecv(0, boost::mpi::any_tag, boost::mpi::get_content(gen_number_dvs));
                            boost::mpi::status s_job = r_job.wait();

                            print_waiting = true;
                            if (this->do_log > this->OFF)
                                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                                 << " received " << gen_number_dvs << " for individual/job number " << s_job.tag() << std::endl;
                            jobs_2_do.insert(std::make_pair(s_job.tag(), gen_number_dvs));
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
                        is_jobs_2_receive = false;
                    }
                }


                if (in_generation)
                {
                    if (!jobs_2_do.empty())
                    {
                        // Now run the job at the front of the queue
                        Jobs2DoMultiSet::iterator job_2_do = jobs_2_do.begin();
                            //calc objective
                            if (!do_save)
                            {
                                objs_and_constraints = eval(std::get<1>(job_2_do->second), std::get<2>(job_2_do->second));
                            }
                            else
                            {
                                boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(job_2_do->first));
                                if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
                                objs_and_constraints = eval(std::get<1>(job_2_do->second), std::get<2>(job_2_do->second), save_ind_dir);
                            }

                            std::get<0>(objs_constraints_gen_num_dvs) = objs_and_constraints.first;
                            std::get<1>(objs_constraints_gen_num_dvs) = objs_and_constraints.second;
                            std::get<2>(objs_constraints_gen_num_dvs) = std::get<0>(job_2_do->second);
                            std::get<3>(objs_constraints_gen_num_dvs) = std::get<1>(job_2_do->second);
                            std::get<4>(objs_constraints_gen_num_dvs) = std::get<2>(job_2_do->second);
                            boost::mpi::request rq;
                            jobs_done.push_front(std::make_tuple(rq, job_2_do->first, objs_constraints_gen_num_dvs));
                            std::tuple<boost::mpi::request, int,  JobsDoneInfo> & job_done = jobs_done.front();

                        if (this->do_log > this->OFF)
                                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                                 << " sending " << std::get<2>(job_done) << " for individual/job number " << std::get<1>(job_done) << std::endl;
                            std::get<0>(job_done) = world.isend(0, std::get<1>(job_done), boost::mpi::get_content(std::get<2>(job_done)));

                            jobs_2_do.erase(job_2_do);
                    }

                }


                // keep the number of jobs in the queue under control
                    bool do_continue = false;
                    do{
                        do_continue = false;
                        if (jobs_2_do.size() > MAX_JOBS_ON_QUEUE)
                        {
                            Jobs2DoMultiSet::iterator old_job_2_do = jobs_2_do.end();
                            if (std::get<0>(old_job_2_do->second) < gen_num)
                            {
                                jobs_2_do.erase(old_job_2_do);
                                do_continue = true;
                            }
                        }
                    } while (do_continue);



                // Test sends back, and remove those completed.
                boost::optional<boost::mpi::status> os;
                do{
                    os = boost::optional<boost::mpi::status>();
                    if (!jobs_done.empty())
                    {
                        std::tuple<boost::mpi::request, int,  JobsDoneInfo> & job_sending = jobs_done.back();
                        os = std::get<0>(job_sending).test();
                        if (os)
                        {
                            if (this->do_log > this->OFF)
                                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
                                                << " individual/job number " << std::get<1>(job_sending) << " now sent" << std::endl;
                            jobs_done.pop_back();
                        }
                    }
                } while (os);


            }
            if (this->do_log)
            {
                if (logging_file.is_open()) logging_file.close();
            }

            if (this->do_log)
            {
                    if (gen_num>3) deletePreviousLog(log_file_base_name, gen_num-3, log_subdir_name);
            }

        }

    }

};


//class ParallelEvaluatePopServer : public ParallelEvaluatorSeverBase
//{
//
//public:
//    ParallelEvaluatePopServer(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr _problem_defs, std::time_t _timeout_time = 1800)
//    : ParallelEvaluatorSeverBase(_mpi_env, _world, _problem_defs)
//    {
//
//    }
//
//    ~ParallelEvaluatePopServer()
//    {
//        std::ofstream logging_file;
//        if (this->do_log)
//        {
//            std::string file_name = "parallel_eval_server_term.log";
//            log_file = this->log_directory / file_name;
//            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
//            if (!logging_file.is_open()) this->do_log = this->OFF;
//        }
//
//        // Send signal to slaves to indicate shutdown.
////        std::cout << "In destructor" << std::endl;
//        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending terminate" << std::endl;
//        std::string terminate = TERMINATE;
//        boost::mpi::broadcast(world, terminate,0);
//    }
//
//    void
//    evalPop(PopulationSPtr population, boost::filesystem::path save_dir)
//    {
//
//        std::ofstream logging_file;
//        if (this->do_log)
//        {
//            std::string file_name = "parallel_eval_server" + std::to_string(this->gen_num) + ".log";
//            this->log_file = this->log_directory / file_name;
//            logging_file.open(log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
//            if (!logging_file.is_open()) this->do_log = this->OFF;
//        }
//
//        if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " Broadcasting save directory: " << save_dir << std::endl;
//        std::string save_dir_s = save_dir.string();
//        boost::mpi::broadcast(world, save_dir_s,0);
//
//        if (this->do_log > this->OFF) logging_file << "Evaluating population with size: " << population->size() << std::endl;
//
//        //Sanity check - that we can represent each individual by an mpi tag.
//        if (population->populationSize() > (max_tag - 1))
//        {
//            if (this->do_log > this->OFF) logging_file << "problem: max tag too small, population too large for mpi" << std::endl;
//        }
//
//        int individual = 0;
//        std::vector<boost::mpi::request> reqs_out(number_clients);
//        if (this->do_log > this->OFF) logging_file << "Evaluating population using " << number_clients << std::endl;
//        int num_initial_jobs = number_clients;
//        if (num_initial_jobs > population->populationSize()) num_initial_jobs = population->populationSize();
//        for (; individual < num_initial_jobs; ++individual)
//        {
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " << "Evaluating individual " << individual << std::endl;
//            std::get<0>(gen_number_dvs) = this->gen_num;
//            std::get<1>(gen_number_dvs) = (*population)[individual]->getRealDVVector();
//            std::get<2>(gen_number_dvs) = (*population)[individual]->getIntDVVector();
//            int client_id = individual + 1;
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time() << " sending to " << client_id << " individual " << individual << " with "  << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVs: " << decision_vars << std::endl;
//            world.send(client_id, individual, gndv_c);
//        }
////        mpi::wait_all(reqs_out.begin(), reqs_out.end());
//
//        while (individual < population->populationSize())
//        {
//            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
//            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
//            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
//
//            decision_vars.first = (*population)[individual]->getRealDVVector();
//            decision_vars.second = (*population)[individual]->getIntDVVector();
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << s.source() << " individual " << individual << " with " << decision_vars.first.size() << ", " << decision_vars.second.size() << " DVs: " << decision_vars << std::endl;
//            world.send(s.source(), individual, dv_c);
//
//            ++individual;
//        }
//
//        for (int i = 0; i < number_clients; ++i)
//        {
//            boost::mpi::status s = world.recv(boost::mpi::any_source, boost::mpi::any_tag, oc_c);
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " received from " << s.source() << " individual " << s.tag() << " with " << objs_and_constraints << std::endl;
//            (*population)[s.tag()]->setObjectives(objs_and_constraints.first);
//            (*population)[s.tag()]->setConstraints(objs_and_constraints.second);
//
//
////            world.send(s.source(), max_tag, dv_c);
//        }
//
//        for (int i = 0; i < number_clients; ++i)
//        {
//            int client_id = i + 1;
////            std::cout << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " terminate\n";
//            if (this->do_log > this->OFF) logging_file << world.rank() << ": " <<  boost::posix_time::second_clock::local_time()  << " sending to " << client_id << " end of generation" << std::endl;
//            world.isend(client_id, max_tag, dv_c);
//        }
//
//        if (this->do_log)
//        {
//            if (logging_file.is_open()) logging_file.close();
//        }
//
//        if (this->do_log)
//        {
//            if (this->delete_previous_logfile && !this->previous_log_file.empty()) boost::filesystem::remove_all(this->previous_log_file);
//            this->previous_log_file = this->log_file;
//        }
//    }
//
//    void
//    operator()(PopulationSPtr population)
//    {
//        this->evalPop(population, NO_SAVE);
//    }
//
//    void
//    operator()(PopulationSPtr population, boost::filesystem::path & save_dir)
//    {
//        this->evalPop(population, save_dir);
//    }
//};

//class ParallelEvaluatePopClientNonBlocking : public ParallelEvaluatorClientBase
//{
//    ObjectivesAndConstraintsBase & eval;
//
//public:
//    ParallelEvaluatePopClientNonBlocking(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
//    : ParallelEvaluatorClientBase(_mpi_env, _world, _problem_defs), eval(_eval)
//    {
//
//    }
//
//    void
//    operator()()
//    {
//        std::ofstream logging_file;
//        if (this->do_log)
//        {
//            std::string file_name = "parallel_evaluate_client" + std::to_string(world.rank()) + "_nonblock" + std::to_string(++this->gen_num) + std::string(".log");
//            this->log_file = this->log_directory / file_name;
//            logging_file.open(this->log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
//            if (!logging_file.is_open()) this->do_log = this->OFF;
//        }
//
//        bool do_continue = true;
//        bool in_generation = true;
//        boost::mpi::request rq;
//        bool first_time = true;
//        std::string save_dir_s;
//        boost::filesystem::path save_dir;
//        bool do_save;
//
//        while (do_continue)
//        {
//            if (this->do_log > this->OFF)
//                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                 << " Waiting to receive save-dir" << std::endl;
//            boost::mpi::broadcast(world, save_dir_s,0);
//            if (save_dir_s == NO_SAVE)
//            {
//                do_save = false;
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " Not saving evaluation to file" << std::endl;
//                in_generation = true;
//            }
//            else if (save_dir_s == TERMINATE)
//            {
//                in_generation = false;
//                do_continue = false;
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " Terminating" << std::endl;
//                break;
//            }
//            else
//            {
//                do_save = true;
//                save_dir = save_dir_s;
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " Saving evaluation here: " << save_dir_s << std::endl;
//                in_generation = true;
//            }
//
//            while (in_generation)
//            {
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " waiting to receive" << std::endl;
//                boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " received " << decision_vars << " for individual " << s.tag() << std::endl;
//                if (s.tag() == max_tag)
//                {
//                    in_generation = false;
//                    if (this->do_log > this->OFF)
//                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                         << " End of generation" << std::endl;
//                }
//                else
//                {
//                    //calc objective
//                    if (!do_save)
//                    {
//                        objs_and_constraints = eval(decision_vars.first, decision_vars.second);
//                    }
//                    else
//                    {
//                        boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(s.tag()));
//                        if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
//                        objs_and_constraints = eval(decision_vars.first, decision_vars.second, save_ind_dir);
//                    }
//
//                    if (this->do_log > this->OFF)
//                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                         << " sending " << objs_and_constraints << " for individual " << s.tag();
//                    logging_file.flush();
////                                         << std::endl;
//                    if (!first_time)
//                    {
//                        rq.wait();
//
//                    }
//                    else
//                    {
//                        first_time = false;
//                    }
//                    rq = world.isend(0, s.tag(), oc_c);
//                    if (this->do_log > this->OFF)
//                        logging_file << "; now sent"  << std::endl;
//
//                }
//            }
//
//        }
//
//        if (this->do_log)
//        {
//            if (logging_file.is_open()) logging_file.close();
//        }
//
//        if (this->do_log)
//        {
//            if (this->delete_previous_logfile && !this->previous_log_file.empty()) boost::filesystem::remove_all(this->previous_log_file);
//            this->previous_log_file = this->log_file;
//        }
//    }
//};

//class ParallelEvaluatePopClient : public ParallelEvaluatorClientBase
//{
//    ObjectivesAndConstraintsBase & eval;
//
//public:
//    ParallelEvaluatePopClient(boost::mpi::environment & _mpi_env, boost::mpi::communicator & _world, ProblemDefinitionsSPtr  _problem_defs, ObjectivesAndConstraintsBase & _eval)
//        : ParallelEvaluatorClientBase(_mpi_env, _world, _problem_defs), eval(_eval)
//    {
//
//    }
//
//    void
//    operator()()
//    {
//        std::ofstream logging_file;
//        if (this->do_log)
//        {
//            std::string file_name = "parallel_evaluate_client" + std::to_string(world.rank()) + std::to_string(++this->gen_num) + std::string(".log");
//            this->log_file = this->log_directory / file_name;
//            logging_file.open(this->log_file.string().c_str(), std::ios_base::out | std::ios_base::trunc);
//            if (!logging_file.is_open()) this->do_log = this->OFF;
//        }
//
//
//        bool do_continue = true;
//        bool in_generation = true;
////        boost::mpi::request rq;
////        bool first_time = true;
//        std::string save_dir_s;
//        boost::filesystem::path save_dir;
//        bool do_save;
//
//        while (do_continue)
//        {
//            if (this->do_log > this->OFF)
//                logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                 << " Waiting to receive save-dir" << std::endl;
//            boost::mpi::broadcast(world, save_dir_s,0);
//            if (save_dir_s == "no_save")
//            {
//                do_save = false;
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " Not saving evaluation to file" << std::endl;
//                in_generation = true;
//            }
//            else if (save_dir_s == "terminate")
//            {
//                in_generation = false;
//                do_continue = false;
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " Terminating" << std::endl;
//                break;
//            }
//            else
//            {
//                do_save = true;
//                save_dir = save_dir_s;
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " Saving evaluation here: " << save_dir_s << std::endl;
//                in_generation = true;
//            }
//            while (in_generation)
//            {
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " waiting to receive" << std::endl;
//                boost::mpi::status s = world.recv(0, boost::mpi::any_tag, dv_c);
//                if (this->do_log > this->OFF)
//                    logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                     << " received " << decision_vars << " for individual " << s.tag() << std::endl;
//                if (s.tag() == max_tag)
//                {
//                    in_generation = false;
//                    if (this->do_log > this->OFF)
//                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                         << " End of generation" << std::endl;
//                }
//                else
//                {
//                    //calc objective
//                    if (!do_save)
//                    {
//                        objs_and_constraints = eval(decision_vars.first, decision_vars.second);
//                    }
//                    else
//                    {
//                        boost::filesystem::path save_ind_dir = save_dir / ("individual_" + std::to_string(s.tag()));
//                        if (!boost::filesystem::exists(save_ind_dir)) boost::filesystem::create_directories(save_ind_dir);
//                        objs_and_constraints = eval(decision_vars.first, decision_vars.second, save_ind_dir);
//                    }
//
//                    if (this->do_log > this->OFF)
//                        logging_file << world.rank() << ": " << boost::posix_time::second_clock::local_time()
//                                         << " sending " << objs_and_constraints << " for individual " << s.tag()
//                                         << std::endl;
//
//                    world.send(0, s.tag(), oc_c);
//
//                }
//            }
//
//        }
//
//        if (this->do_log)
//        {
//            if (logging_file.is_open()) logging_file.close();
//        }
//
//        if (this->do_log)
//        {
//            if (this->delete_previous_logfile && !this->previous_log_file.empty()) boost::filesystem::remove_all(this->previous_log_file);
//            this->previous_log_file = this->log_file;
//        }
//    }
//};


#endif /* ParallelEvaluator_h */
