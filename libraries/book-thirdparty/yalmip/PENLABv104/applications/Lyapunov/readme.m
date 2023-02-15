% We solve the problem of stability of a time invariant linear (discrete)
% system using Lyapunov theory. The problem is formulated as a LMI system
% A'*P + P*A < 0 , P > I (continuous time)
% or
% A'*P*A - P < 0 , P > I (discrete time)
% We minimize the trace of P.

%Example LYAPU
A=[0 1 0;0 0 1;-1 -2 -3];
penm = lyapu(A);
prob = penlab(penm); 
solve(prob);
P=prob.Y{1};

%Example DLYAPU
A = [.7 -.2 -.1; .5 .4 0; 0 -.5 .9];
penm = dlyapu(A);
prob = penlab(penm); 
solve(prob);
P=prob.Y{1};

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 02 Dec 2013