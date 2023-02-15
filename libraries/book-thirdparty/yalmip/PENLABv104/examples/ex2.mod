#
# Ampl model file for problem Example 2
#

var x {1..2};

minimize  f: -2*x[1] + x[2];

subject to g1: -(1-x[1])^3 + x[2] <= 0;
subject to g2: -x[2] - 0.25*x[1]^2 + 1 <= 0;

