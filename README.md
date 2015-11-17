# parallel nsgaII backend
The backend engine of a parallelised NSGAII making use of MPI (written in C++)
Work supported with funding from the Bushfire and Natural Hazard Cooperative Research Centre

NSGA-II is a fast and elitist multiobjective genetic algorithm. What this means is
1. It is a software routine that searches for a good solution to an optimisation problem, in other words, it is a metaheuristic

2. Optimisation problems are generally problems that try to minimise or maximise the value of one or several things. For example, a business might want to maximise it's profit, a designer might want to minimise the cost of their product, an environmental manager might want to minimise the amount of pollution being discharged from a site, you might want to maximise your lifestyle outcomes. 

3. Often, people are facing tasks where there are multifaceted objectives. For example, a business may not only care about their profit, but also the work culture of their offices, their environmental credentials, and their public image, they may also have long-term business objectives that clash with short-term profit. This is an example of multiplke objectives: and NSGA-II is a multiobjective optimiser - it will search for the optimal trade-offs amongst competing objectives, called a Pareto front (https://en.wikipedia.org/wiki/Pareto_efficiency). In other words, the NSGA-II will find multiple solutions. Each of these solutions will represent alternative priorities (or weightings, or tradeoffs) between the objectives. 

4. A number of factors affect objectives. For example, in maximising water quality in watercourses, this objective is influenced by the number of households, inustries and commerical sites that are connected to water treatment plants, the extent of the treatment that occurs at these plants, whether action is taken in the watercourses flow path to enhance water quality (such as constructing swales, wetlands, trash racks, aerators, water treatment plants), and whether public enducation campaigns are undertaken or financial incentives are given to encourge individuals to reduce their water quality burden. Any variable, such as the ones listed above, that the decision maker/designer has ability to control in order to reach their objectives is called a decision variable. The NSGA-II searches for good solutions to the optimisation problem by exploring the effect these decision variables have on the objective function.

5. The final aspect to optimisation problem is constraints. As an example of a constraint, consider the design of an information communication network, such as a computer network. The designer of a network will need to minimise the cost of the network. However, they may also have a constraint that the network must support a minimum bandwidth requirement --- that is, it must atleast be able to transmit data at a sufficiently high rate, with requirements for, say video transmission being much higher than text.

6. Solving many optimisation problems is not trivial. As an example, designing pipe networks for water distribution usually involves specifying the diameters for dozens, if not hundreds of pipes. There may be upward of five different pipe diameters available to use for each of these pipes (usually based on what your pipe supplier/manufacturer stocks/builds). In addition, pumps may also need to be specified as part of the design. The number of alternative pipe and pump combinations is usually larger than the number of atoms present within the planet earth. There is no known deterministic method for finding the optimal combination of pipes/pumps for this problem. This is why metaheurisitics are often used for these types of problems

7. To use the NSGA-II, developed here, you need to have some software/code that will calculate your objectives and constraints. This may be a simple function, or a large integrated model that simulates the entire system that is being optimised.

8. In searching for optimal solutions, the NSGA-II needs to evaluate the objectives and constraints thousands of times (on the order of 10,000 to 100,000 times is typical, depending on the difficulty of the problem). Obviously, there are linmitations on how long it takes a computer to evaluate these objectives and constraints, as we usually only have a few hours, days or up to a few months in which to solve these optimisation problems.

9. As computers have become more powerful, researchers and practitioners have developed more and more complex and involved methods for calculating objective functions. In addition, objective functions that were once took too long to solve, are now able to be computed much faster. Even so, one of the limitations of metaheuritsics such as NSGA-II is the length of time required to solve an objective function. Fortunately, 'population-based' metaheuristics such as NSGA-II are easily parallelised, and able to take advantage of modern computing hardware; in recent years, greater computing power has been provided more by increasing the number of processors on a chip, rather than making faster processors.

10. The NSGA-II developed here using a software framework known as MPI to enable parallelisation. This allows the NSGA-II to be parallelised across multiple cpus. It can be used on distributed computing networks (such as clusters) or shared-memory machines (such as your typical multicore desktop computer).

11. The NSGA-II developed here can be used across a braod range of optimisation problems, although most of our applications have been in the design of water infrastructure (supply, stormwater quality, flood control), environmental management (environmental flows), landuse policy development, and maintenance scheduling.

If you publish on this work, you are requested to cite:


and any of the following papers relevant to your work:
