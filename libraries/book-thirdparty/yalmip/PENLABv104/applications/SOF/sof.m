function [K]=sof(fname)
% SOF - Static Output Feedback via polynomial matrix inequalities
%
% We solve feasibility problem for examples from the COMPLib library
% The user has to install COMPLib from http://www.compleib.de/
% and add it to the MATLAB path

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

pmidata=complib2pmi(fname);
penm=pmi_define(pmidata);
prob=penlab(penm);
%penm.opts.outlev=3;
solve(prob);
x = prob.x;

[A,B1,B,C1,C]=COMPleib(fname);
K = reshape(x(1:end-1),size(B,2),size(C,1));
eig(A+B*K*C)