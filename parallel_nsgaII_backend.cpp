/*
 *  parallel_nsgaII_backend.cpp
 *  parallel-nsgaII-backend
 *
 *  Created by a1091793 on 18/11/2015.
 *  Copyright © 2015 University of Adelaide. All rights reserved.
 *
 */

#include <iostream>
#include "parallel_nsgaII_backend.hpp"
#include "parallel_nsgaII_backendPriv.hpp"

void parallel_nsgaII_backend::HelloWorld(const char * s)
{
	 parallel_nsgaII_backendPriv *theObj = new parallel_nsgaII_backendPriv;
	 theObj->HelloWorldPriv(s);
	 delete theObj;
};

void parallel_nsgaII_backendPriv::HelloWorldPriv(const char * s) 
{
	std::cout << s << std::endl;
};

