% same example as bmi_ex3.m but using the new BMI structure
%
% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

clear bmidata;
bmidata.name='Example 3 from BMI manual via BMI input';
bmidata.Nx=3;
bmidata.Na=1;

% objective
bmidata.c=[0; 0; 1];   % dim (Nx,1), coefficients of the linear objective function

% box constraints, last one is unconstrained
bmidata.lbx=[-0.5; -3; -Inf];
bmidata.ubx=[   2;  7;  Inf];

% one matrix constraint
nMat=5;
maxOrder=2;
Q=cell(nMat,1);
midx=zeros(maxOrder,nMat);
Q{1} = sparse([10 .5 2;.5 -4.5 0; 2 0 0]);      % A_0
midx(:,1)=[0;0];
Q{2} = -sparse([9 .5 0; .5 0 -3; 0 -3 -1]);     % A_1
midx(:,2)=[1;0];
Q{3} = -sparse([-1.8 -.1 -.4; -.1 1.2 -1; -.4 -1 0]);  % A_2
midx(:,3)=[2;0];
Q{4} = speye(3,3);                                     % A_3
midx(:,4)=[3;0];
Q{5} = -sparse([0 0 2;0 -5.5 3; 2 3 0]);               % K_12
midx(:,5)=[1;2];
bmidata.A{1}.midx=midx;
bmidata.A{1}.Q=Q;

