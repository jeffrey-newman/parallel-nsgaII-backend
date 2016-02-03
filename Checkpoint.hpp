//
//  Checkpoint.hpp
//  NSGA-Parallel-Backend
//
//  Created by a1091793 on 25/11/2015.
//
//

#ifndef Checkpoint_h
#define Checkpoint_h

class Checkpoint
{
    int max_gen;
    int gen_number;
    
public:
    Checkpoint(int _max_gen)
    : max_gen(_max_gen), gen_number(0)
    {
        
    }
    
    bool
    operator()(PopulationSPtr population)
    {
        ++gen_number;
        if (gen_number > max_gen)
        {
            return false;
        }
        return true;
    }
};

#endif /* Checkpoint_h */
