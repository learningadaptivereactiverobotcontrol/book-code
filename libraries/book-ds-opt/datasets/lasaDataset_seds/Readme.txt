%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                LASA Handwriting Dataset, version 2.0

DataSet: contains a library of 2D handwriting motions recorded from 
         Tablet-PC. For each motion, the user was asked to draw 7 
         demonstrations of a desired pattern, by starting from different 
         initial positions (but fairly close to each other) and ending to 
         the same final point. These demonstrations may intersect each other.
         In total a library of 30 human handwriting motions were collected, 
         of which 26 each correspond to one single pattern, the remaining 
         four motions each include more than one pattern (called Multi Models). 
         Without loss of generality, for all handwriting motions (shapes), 
         the target is by definition set at (0, 0). Demonstrations are saved 
         as '.mat' file and contains two variables:

            o dt: the average time steps across all demonstrations
            o demos: A structure variable containing ncessary informations
              about all demonstrations. The variable 'demos' has the following
              format:
              - demos{n}:     Information related to the n-th demonstration.
              - demos{n}.pos: 2 x 1000 matrix representing the motion in 2D
                              space. The first and second rows correspond to
                              x and y axes in the Cartesian space, respectively.
              - demons{n}.t:  1 x 1000 vector indicating the corresponding time
                              for each datapoint (i.e. each column of demos{n}.pos).
              - demos{n}.vel: 2 x 1000 matrix representing the velocity of the motion.
              - demos{n}.acc: 2 x 1000 matrix representing the acceleration of the motion.

Run the file 'demo_Plot_models.m' to get an idea about how the motions look like.

Note: When recording the data, we put the constraint to represent each demonstrion
with 1000 datapoint. To handle different final time, we have thus interpolated 
between the collected datapoints. As a consequence, you will notice a different 
dt for each demonstration.

Since each demonstration trajectory may have different final time,
you will also notice a different dt.

This library of motion is free for non-commercial academic use. The library 
must not be modified or distributed without prior permission of the authors.
Please acknowledge the authors in any academic publications that have made 
use of this library or part of it. Please use this BibTex for reference:

 S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical 
 Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.

To get latest upadate of the software please visit
                          http://lasa.epfl.ch/khansari

Please send your feedbacks or questions to:
                          mohammad.khansari_at_epfl.ch
