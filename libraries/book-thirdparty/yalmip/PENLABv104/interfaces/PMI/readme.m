% *Polynomial matrix inequalities*
% 
% We want to solve an optimization problem with constraints in the form of
% polynomial matrix inequalities:
%    min   c'x + 1/2 x'Hx
%    s.t.  lbg <= B*x <= ubg
%          lbx <=  x  <= ubx 
%          A_k(x)>=0     for k=1,..,Na
% where
%          A_k(x) = sum_i  x(multi-index(i))*Q_i
%
% The problem is stored in a structure |pmidata| described below
% (obviously, the name of the structure is optional). For instance, you can
% create a sample structure by calling
%
pmi_ex3
%
% or by loading  one of the stored SOF examples, e.g.
%
load pmi_AC1
%
% Once the structure is in the memory, all the user has to
% do to solve the problem is to call the following sequence of commands:
% 
penm=pmi_define(pmidata);
prob=penlab(penm);
prob.solve();
%
% Example of A_k(x):
%
%          A_k(x) = Q_1 + x_1*x_3*Q_2 + x_2*x_3*x_4*Q_3
%
% where multi-indices are  
%
%             midx_1 = 0       (absolute term, Q_1)
%             midx_2 = [1,3]   (bilinear term, Q_2)
%             midx_3 = [2,3,4] (term for Q_3)
%
% List of elements of the user structure |pmidata|
%
%   name ... [optional] name of the problem
%   Nx ..... number of primal variables
%   Na ..... [optional] number of matrix inequalities (or diagonal blocks
%            of the matrix constraint)
%   xinit .. [optional] dim (Nx,1), starting point
%   c ...... [optional] dim (Nx,1), coefficients of the linear obj. function,
%            considered a zero vector if not present
%   H ...... [optional] dim (Nx,Nx), Hessian for the obj. function,
%            considered a zero matrix if not present
%   lbx,ubx. [optional] dim (Nx,1) or scalars (1x1), lower and upper bound
%            defining the box constraints
%   B ...... [optional] dim(Ng,Nx), matrix defining the linear constraints
%   lbg,ubg. [optional] dim (Ng,1) or scalars, upper and lower bounds for B
%
%   A ...... if Na>0, cell array of A{k} for k=1,...,Na each defining 
%            one matrix constraint; let's assume that A{k} has maximal
%            order maxOrder and has nMat matrices defined, then A{k} should 
%            have the following elements:
%              A{k}.Q - cell array of nMat (sparse) matrices of the same
%                 dimension
%              A{k}.midx - (maxOrder x nMat) matrix defining the multi-indices
%                 for each matrix Q; use 0 within the multi-index to reduce
%                 the order
%            for example, A{k}.Q{i} defines i-th matrix to which belongs
%            multi-index  A{k}.midx(:,i). If midx(:,i) = [1;3;0], it means
%            that Q_i is multiplied by x_1*x_3 within the sum.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013