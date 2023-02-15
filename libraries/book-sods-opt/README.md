
# SESODS_lib

Matlab toolbox to learn a GMM based first and second order dynamical system. This package is developed to provide several approximations LPV based dynamical systems. All the systems are based on Gaussian Mixture models. Please cite this paper if you use this package:

# Dependences 

- Yalmip: https://github.com/yalmip/YALMIP
  - Convex and Non convex solvers
    - Personally, I use sedumi, PENLAB and mosek solvers. I've also used Cplex and it is obviously Nice:D 
    
# Features:
- GMR based stable first order dynamical systems.
- GMR based stable second order dynamical systems.
- Several different implementations: Convex or Non convex solvers.
- Signal processing and calculating velocity and acceleration from positions are also included.

# How to run
- If you want to use fmincon solver, you need to run [Main_File.m](https://github.com/sinamr66/SESODS_lib/blob/master/Non_convex/Main_File.m)
  - This package constructs a seond order stable DS.
  - It depends on your data-set, sometimes it works better than Yalmip.
- If you want to use Yalmip interface, you need to run [Stable_systems_analysis.m](https://github.com/sinamr66/SESODS_lib/blob/master/Convex/Stable_systems_analysis.m)
  - This package constructs a first order stable DS.
  - The problem can be formulated as a convex or non-convex optimization. You can check the options in the code. 
- If you want to use Yalmip interface, you need to run [Stable_systems_analysis_Second_order.m](https://github.com/sinamr66/SESODS_lib/blob/master/Convex/Stable_systems_analysis_Second_order.m)
  - This package construct a second order stable DS.
  - The problem is formulated as a non-convex problem. It is easy to formulate it as a convex problem but I have not done it.
  

This work has been done in collaboration with Alireza Karimi. 
For more information contact Sina Mirrazavi. 
## Copyright
Please cite the following thesis, if you are using this toolbox:


@article{MirrazaviSalehian:255170,
      title = {Compliant control of Uni/ Multi- robotic arms with  dynamical systems },
      author = {Mirrazavi Salehian, Seyed Sina},
      institution = {IMT},
      publisher = {EPFL},
      pages = {169},
      year = {2018},
}

