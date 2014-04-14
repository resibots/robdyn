robdyn
======

A simple C++ wrapper around ODE to make dynamic simulators for robots. Example of the work: http://pages.isir.upmc.fr/~mouret/website/

Don't hesitate to contribute! 

**If you use this software in an academic article, please cite one of the related team's article:**
*  Koos, S. and Mouret, J.-B. and Doncieux, S. (2013). The Transferability Approach: Crossing the Reality Gap in Evolutionary Robotics.
_IEEE Transactions on Evolutionary Computation._ Vol 17 No 1 Pages 122 - 145.
* Koos, S. and Cully, A. and Mouret, J.-B. (2013). Fast Damage Recovery in Robotics with the T-Resilience Algorithm. _International Journal of Robotics Research._ Vol 32 No 14 Pages 1700-1723.
* Doncieux, S. and Mouret, J.B. (2013). Behavioral Diversity with Multiple Behavioral Distances. _Proc. of IEEE Congress on Evolutionary Computation,_ 2013 (CEC 2013). Pages 1-8.
*  Cully, A. and Mouret, J.-B. (2013). Behavioral Repertoire Learning in Robotics. _Genetic and Evolutionary Computation Conference (GECCO)._ Pages 175-182.
This software is developped in ISIR (http://isir.upmc.fr). 



Dependencies
------------
(the current Debian/Ubuntu packages should work).
- OpenSceneGraph: http://www.openscenegraph.org/‎
- ODE: http://www.ode.org
- boost: http://www.boost.org
- eigen2 : http://eigen.tuxfamily.org/index.php?title=Main_Page

Compiling
---------
- ./waf configure
- ./waf install

### Useful options:
- --boost-libs=
- --boost-includes=
- --help
- --eigen2=
- --disable_osg=yes


Credit
------
This software was mainly written by Jean-Baptiste Mouret (mouret@isir.upmc.fr), with contributions from Sylvain Koos (sylvain.koos@gmail.com) and Antoine Cully (cully@isir.upmc.fr).

