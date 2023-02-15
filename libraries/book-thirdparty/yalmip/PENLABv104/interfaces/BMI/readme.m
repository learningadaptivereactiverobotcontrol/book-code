% *Interface for SDP with Bilinear Matrix Inequalities (BMI)*
% 
% We want to solve an optimization problem with constraints in the form of
% bilinear (or linear) matrix inequalities:
%    min   c'x + 1/2 x'Hx
%    s.t.  lbg <= B*x <= ubg
%          lbx <=  x  <= ubx 
%          A_k(x)>=0     for k=1,..,Na
% where
%          A_k(x) = A_0 + sum_i x_i*A_i + sum_ij x_i*x_j*K_ij
% or written in the same notation as in PMI using multi-indices
%          A_k(x) = sum_i  x(multi-index(i))*Q_i
%
% The problem is stored in a structure |bmidata| described below
% (obviously, the name of the structure is optional). Interface BMI is
% fully compatible with PMI thus |bmidata| can be used in PMI and
% |pmidata| in BMI as long as the order of matrix constraints is 
% at most 2. However, BMI is optimized for linear and bilinear matrix
% constraints and thus should be preferred to PMI for these orders.
%
% For instance, you can create a sample structure by calling
%
bmi_ex3
%
% or by loading one of the stored examples, e.g.
%
load ../../datafiles/bmi_f4e.mat
%
% Once the structure is in the memory, all the user has to
% do to solve the problem is to call the following sequence of commands:
% 
penm = bmi_define(bmidata);
prob = penlab(penm);
prob.solve();
%
% Example of A_k(x):
%
%          A_k(x) = Q_1 + x_1*x_3*Q_2 + x_2*x_2*Q_3
%
% where multi-indices are  
%
%             midx_1 = 0       (absolute term, Q_1)
%             midx_2 = [1,3]   (bilinear/quadratic term, Q_2)
%             midx_3 = [2,2]   (bilinear/quadratic term, Q_3)
%
% List of elements of the user structure |bmidata| (as noted above,
% it is the same as |pmidata| only with restricted maxOrder<=2)
%
%   name ... [optional] name of the problem
%   Nx ..... number of primal variables
%   Na ..... [optional] number of matrix inequalities (or diagonal blocks
%            of the matrix constraint)
%   xinit .. [optional] dim (Nx,1), starting point
%
%   c ...... [optional] dim (Nx,1), coefficients of the linear obj. function,
%            considered a zero vector if not present
%   H ...... [optional] dim (Nx,Nx), Hessian for the obj. function,
%            considered a zero matrix if not present
%
%   lbx,ubx. [optional] dim (Nx,1) or scalars (1x1), lower and upper bound
%            defining the box constraints
%   B ...... [optional] dim(Ng,Nx), matrix defining the linear constraints
%   lbg,ubg. [optional] dim (Ng,1) or scalars, upper and lower bounds for B
%
%   A ...... if Na>0, cell array of A{k} for k=1,...,Na each defining 
%            one matrix constraint; let's assume that A{k} has maximal
%            order maxOrder=2 and has nMat matrices defined, then A{k} should 
%            have the following elements:
%              A{k}.Q - cell array of nMat (sparse) matricies of the same
%                 dimension
%              A{k}.midx - matrix maxOrder x nMat defining the multi-indices
%                 for each matrix Q; use 0 within the multi-index to reduce
%                 the order
%            for example, A{k}.Q{i} defines i-th matrix to which belongs
%            multi-index  A{k}.midx(:,i). If midx(:,i) = [1;3], it means
%            that Q_i is multiplied by x_1*x_3 within the sum;
%            midx(:,j)=[0;0], it means that Q_j is absolute term.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013
