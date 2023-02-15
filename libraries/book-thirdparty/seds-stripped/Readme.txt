SEDS Package: version 1.95 issued on 12 February 2013

This packages contains the SEDS learning algorithm presented in the following
paper:

 S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical 
 Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

The program is free for non-commercial academic use. Please contact the
author if you are interested in using the software for commercial purposes.
The software must not be modified or distributed without prior permission
of the authors. Please acknowledge the authors in any academic publication
that have made use of this code or part of it. Please use the aforementioned
paper for reference.

To get latest update of the software please visit
                          http://lasa.epfl.ch/khansari

Please send your feedbacks or questions to:
                          mohammad.khansari_at_epfl.ch


This source code include two matlab functions: 'demo_Plot_Results.m' and 
'demo_SEDS_Learning.m', and 4 subdirectories: 'SEDS_lib', 'GMR_lib', 'models',
and 'Doc'.

demo_Plot_Results: a matlab function illustrating the obtained results using SEDS
                   for the 24 handwriting motions provided in 'models' folder.

demo_Plot_Learning: a matlab script illustrating how to use SEDS_lib to learn
                    an arbitrary model from a set of demonstrations.

SEDS_lib: contains code which implements SEDS. SEDS library also depends on
          GMR library. See the document 'Doc/SEDS_Slides.pdf' for further 
	  information about how to use this library.

SEDS_Cpp_lib: A ROS package to use SEDS in realtime control. See the document 
	      'Doc/SEDS_Slides.pdf' for further information about how to use 
	      this package.

Extensions: Extensions to SEDS package developed by myself or others. Currently, 
            there are two packages: Coupled Dynamical Systems (CDS), and 
            Dynamical systems-based obstacle avoidance.Currently, the SEDS ROS 
            package only provides an estimate of the desired control policy 
            (e.g. an estimate of the velocity) based on the current situation of
            the robot. The inclusion of the training phase in the ROS package is
            a work under progress.

GMR_lib: A library for Gaussian Mixture Model that is called from SEDS library

models: contains a library of 24 handwriting motions recorded from Tablet-PC.

Doc: Includes 3 documents:
     1) SEDS_Slides.pdf describes the theory behnid SEDS, detailed information
        about how to use SEDS MATLAB and ROS packages, and a description of 
	extension packages based on SEDS.
     
     2) SEDS_Overview.pdf provides a very general overview of SEDS (without any
	mathematical detail) and situate it among the existing approaches in the
	literature.

     3) Khansari_Billard_SEDS_Derivatives.pdf is a technical report about how the
	derivatives of the SEDS optimization is computed. You do NOT need to read
	this report if you only want to use SEDS. Reading of this report is only 
	useful if you want to develop the SEDS.

When running the demos, it is assumed that your current directory is the
SEDS_package directory. Otherwise, you should manually add both the 'SEDS_lib'
and 'GMR_lib' directories to the matab path.

Improvement with respect to previous version:
- Adding SEDS ROS package
- Adding the extension libraries
- Adding the ability to optimize a SEDS model with the objective function of 
  minimizing the angle between demonstrations velocity and their estimations.
- Providing a more detailed documentation
- Fixing minor bugs