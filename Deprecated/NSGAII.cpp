////
//// Created by a1091793 on 12/6/17.
////
//
////
////  NSGAII.hpp
////  NSGA-Parallel-Backend
////
////  Created by a1091793 on 24/11/2015.
////
////
//
//#include "NSGAII.hpp"
//
//#include <iostream>
//#include <functional>
//
//#include <boost/scoped_ptr.hpp>
//#include "DebsCrowdingDistance.hpp"
//
//#ifdef WITH_VTK
//#include "Checkpoints/PlotFronts.hpp"
//#endif
//
//
//template <typename RNG>
//NSGAII<RNG>::NSGAII(RNG & _random_number_generator, ObjectivesAndConstraintsBase & eval)
//        : random_number_generator(_random_number_generator), default_evaluator(eval), pop_eval(default_evaluator), selection(_random_number_generator), /*do_visualise(OFF),*/ do_log(OFF), log_stream(std::cout)
//{
//
//}
//
//template <typename RNG>
//NSGAII<RNG>::NSGAII(RNG & _random_number_generator, EvaluatePopulationBase & _pop_eval)
//        : random_number_generator(_random_number_generator), default_evaluator(dummy_eval), pop_eval(_pop_eval), selection(_random_number_generator), /*do_visualise(OFF),*/ do_log(OFF), log_stream(std::cout)
//{
//
//}
//
//template <typename RNG> void
//NSGAII<RNG>::add_checkpoint(CheckpointBaseSPtr chkpnt_2_add)
//{
//    my_checkpoints.addCheckpoint(chkpnt_2_add);
//}
//
//template <typename RNG> void
//NSGAII<RNG>::log(std::ostream & _stream, Log _val)
//{
//    do_log = _val;
//    if (do_log > OFF)
//    {
//        is_fstream = false;
//        log_stream = _stream;
//    }
//}
//
//template <typename RNG> void
//NSGAII<RNG>::log(std::ofstream & _stream, boost::filesystem::path _f_path, Log _val)
//{
//    do_log = _val;
//    if (do_log > OFF)
//    {
//        is_fstream = true;
//        f_path = _f_path;
//        log_stream = _stream;
//    }
//}
//
//template <typename RNG> MutationBase &
//NSGAII<RNG>::getRealMutationOperator()
//{
//    return (mutation.getRealMutationOperator());
//}
//
//template <typename RNG> MutationBase &
//NSGAII<RNG>::getIntMutationOperator()
//{
//    return (mutation.getIntMutationOperator());
//}
//
//template <typename RNG> PopulationSPtr
//NSGAII<RNG>::operator()(PopulationSPtr parents)
//{
//    PopulationSPtr children( (Population *) NULL);
//    pop_eval(parents);
//
//    int no_gens = 0;
//
//    do {
//
//        if (is_fstream == true)
//        {
//            std::ofstream * fstreamPtr = dynamic_cast<std::ofstream *>(&(log_stream.get()));
//            fstreamPtr->close();
//            fstreamPtr->open(f_path.c_str(), std::ios_base::out | std::ios_base::trunc);
//            if (!fstreamPtr->is_open()) do_log = OFF;
//        }
//        if (do_log > OFF)  log_stream.get() << "Generation: " << ++no_gens << "\n";
//        if (do_log > OFF)  log_stream.get() << "parents: \n" << parents;
//
//        children = selection(parents);
//
//        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter selection: \n" << children;
//
//        crossover(children);
//
//        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter crossover: \n" << children;
//
//        mutation(children);
//
//        if (do_log > OFF)  log_stream.get() << "\n\n\nAfter mutation: \n" << children;
//
//        pop_eval(children);
//
//        children = merge_calc_front_and_dist(parents, children);
//
//        if (do_log > OFF)  log_stream.get() << "After merge: \n" << children;
//
//        parents = children;
//
//    } while (my_checkpoints(children));
//
//    return (children);
//
//}
//
////friend class boost::serialization::access;
//template<typename RNG> template <typename Archive>
//void NSGAII<RNG>::serialize(Archive & ar, const unsigned int version)
//{
//    ar & BOOST_SERIALIZATION_NVP(crossover);
//    ar & BOOST_SERIALIZATION_NVP(mutation);
////            ar & BOOST_SERIALIZATION_NVP(my_checkpoints);
//    ar & BOOST_SERIALIZATION_NVP(max_gen);
////            ar & BOOST_SERIALIZATION_NVP(do_visualise);
//    ar & BOOST_SERIALIZATION_NVP(pop);
//
//}
//
//#include <random>
//
//// No need to call this TemporaryFunction() function,
//// it's just to avoid link error.
//void TemporaryFunction ()
//{
//    NSGAII<std::mt19937> TempObj;
//}
//
