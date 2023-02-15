#
# Ampl model file for problem Example 1
#

var x {1..3};

minimize  f:  x[1]^2 + 4*x[2]^2 - x[3]^2 + x[1]*x[2] - 2*x[1]*x[3];

subject to box {i in 1..3}: x[i] >= 0;
subject to g: 4 - (x[1]^2 + x[2]^2 + x[3]^2) <= 0;
subject to h: 2*x[1] + 6*x[2] + 4*x[3] - 24 = 0;

