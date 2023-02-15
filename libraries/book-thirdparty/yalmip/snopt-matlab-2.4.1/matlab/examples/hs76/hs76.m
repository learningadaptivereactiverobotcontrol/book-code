function [x,Obj,INFO] = hs76
% Matlab example quadratic problem
% This example calls SQOPT.
%
%   HS 76
%
%           min  f'x + 1/2 * x'Hx
%
%   subject to    x(1) + 2*x(2) +   x(3) + x(4) <= 5
%               3*x(1) +   x(2) + 2*x(3) - x(4) <= 4
%                      -   x(2) - 4*x(3)        <= -1.5
%                  x >= 0
%

% Add path to SQOPT matlab files
addpath([pwd,'/../../'], '-end');

sqscreen('on');
sqprint('hs76.out');

hs76.spc = which('hs76.spc');
sqspec(hs76.spc);
sqseti('Major Iteration limit', 250);


% Set up the problem
m  = 3;
n  = 4;
x0 = zeros(n,1);
x0 = [ .5; .5; .5; .5 ];

% Linear objective term
f = [ -1 -3 1 -1 ]';

% Linear constraint matrix
A = [ 1  2  1  1 ;
      3  1  2 -1 ;
      0 -1 -4  0 ];

al = [];  % No lower bounds A*x
au = [ 5; 4; -1.5 ];

% Lower and upper bounds on x >= 0
xl = zeros(n,1);
xu = []; % No upper bounds on x

% Solve the problem.
[x,Obj,INFO,lambda,states,output] = sqopt(@hs76Hx, f, x0, xl, xu, A, al, au);


sqprint('off');
sqscreen('off');
sqend;


function Hx = hs76Hx(x)
H = [  2  0  -1  0  ;
       0  1   0  0  ;
      -1  0   2  1  ;
       0  0   1  1 ];

Hx = H*x;




