------------------------------------------------------------------------
PenLab, version 1.04 (20140125)
------------------------------------------------------------------------

PENLAB is a free open source software package implemented in MATLAB(r)
for nonlinear optimization, linear and nonlinear semidefinite
optimization and any combination of these. The main attention was given
to clarity of the code rather than tweaks to improve its performance.
This should allow users to better understand the code and encourage them
to edit and develop the algorithm further.

As such, PENLAB is particularly suitable for teaching and research      
purposes. For production use we recommend a tuned parallel              
implementation within NAG Libraries which will be available soon.       

PENLAB is distributed under GNU General Public License 3.0 and should   
be supported on all MATLAB versions starting from R2008a.               

Authors: Jan Fiala, Michal Kocvara, Michael Stingl

We will be more than happy if you sent your feedback to:
   jan@nag.co.uk or m.kocvara@bham.ac.uk.

Project homepage: http://web.mat.bham.ac.uk/kocvara/penlab/index.html

------------------------------------------------------------------------
Quick Start Guide
------------------------------------------------------------------------

(1) Unpack the PenLab distribution package to any directory.

(2) In Matlab, move to the PenLab directory and call
    >> install
    (this will add PenLab subdirectories to the Matlab path)

(3) Test the package by calling (optional)
    >> penlabtest

(4) Solve your first NLP example (Hock-Schittkowski problem 16),
    >> penm = ex3_define();
    >> prob = penlab(penm);
    >> prob.solve();
    >> prob.x
    (the solution should be [ 0.5; 0.25 ], objective 0.25)

(5) Solve your first linear SDP problem
    >> sdpdata = readsdpa('datafiles/control1.dat-s');
    >> penm = sdp_define(sdpdata);
    >> prob = penlab(penm);
    >> prob.solve();

(6) Read the the technical report
      J. Fiala, M. Kocvara, M. Stingl - PENLAB - a solver for nonlinear
      semidefinite programming; submitted to MPC in October 2013.
      ./tex/penlab_paper/paper.pdf
    and the manual
      ./doc/html/manual.html and ./doc
    to know how you can call PenLab directly or via specilized
    interfaces (SDP, BMI, PMI, NLP) or "applications" (NCM, TTO, ...)

------------------------------------------------------------------------
------------------------------------------------------------------------

