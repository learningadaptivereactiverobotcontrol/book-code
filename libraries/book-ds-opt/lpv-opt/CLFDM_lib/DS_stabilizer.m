function [Xd u] = DS_stabilizer(X,fn_handle,Vxf,rho0,kappa0,varargin)
% Syntax:
%
%       [Xd u] = DS_stabilizer(x,fn_handle,Vxf,rho0,kappa0,varargin)
%
% For a given (unstable) dynamical system f, this function computes a
% corrective command u such that xd = f + u becomes globally asymptotically
% stable. Note that f could be autonomous (i.e. xd = f(x)) or
% non-autonomous (i.e. xd = f(t,x)). 
%
% Inputs -----------------------------------------------------------------
%   o X:       If f is an autonomous DS, then X is d by N matrix
%              representing N different query point(s) (each column of X
%              corresponds to each query point). If f is a non-autonomous
%              function, then X is a (d+1) by N matrix. In this case the
%              last row of X corresponds to time, for example X(d+1,10)
%              corresponds to the time at the 10th query point.
%
%   o fn_handle: This is a function handle that evaluates either f(t,x) or
%                f(x).
%
%   o Vxf:     A structure variable representing the energy function. This
%              structure should follow the format explained in learnEnergy.m
%
%   o rho0, kappa0: These parameters impose minimum acceptable rate of decrease
%                   in the energy function during the motion. It computes
%                   this lower bound from the following class \mathcal{K}
%                   function:
%                           rho(\|x\|) = rho0 * ( 1 - exp(-kappa0 * \|x\|) )
%                   Please refer to page 8 of the paper for more information.
% 
%   o varargin:  An optional variable that provides dt (integration time
%                step) to the function, i.e. varargin{1} = dt, dt>0.
%                Providing dt is useful, especially when using large
%                integration time step. Note that our whole stability proof
%                is based on continuous space assumption. When using large
%                time step, the effect of discretization become more
%                dominant and could cause oscillation. Bt providing dt, we
%                could alleviate this issue.
%
% Outputs ----------------------------------------------------------------
%
%   o Xd:       A d x N matrix providing the output velocity after stabilization,
%               i.e. Xd = f + u
%
%   o u:        A d x N matrix corresponding to the stabilizing command that
%               were generated to ensure stability of the dynamical system.
%               When u(:,i) = 0, it means the function f is stable by
%               itself at that query point, and no stabilizing command was
%               necessary. Note: we provide u as an output just for
%               information, you do NOT need to add it to the output
%               velocity!
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         Copyright (c) 2014 Mohammad Khansari, LASA Lab, EPFL,       %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 
% S.M. Khansari-Zadeh and A. Billard (2014), "Learning Control Lyapunov Function
% to Ensure Stability of Dynamical System-based Robot Reaching Motions." 
% Robotics and Autonomous Systems, vol. 62, num 6, p. 752-765.
%
% To get latest update of the software please visit
%                          http://cs.stanford.edu/people/khansari/
%
% Please send your feedbacks or questions to:
%                          khansari_at_cs.stanford.edu

d = Vxf.d;
if size(X,1) == 2*d
    Xd = X(d+1:2*d,:);
    X = X(1:d,:);
else
    if nargin(fn_handle)==1
        Xd = fn_handle(X);
    elseif nargin(fn_handle)==2
        t = X(d+1,:);
        X(d+1,:) = [];
        Xd = fn_handle(t,X);
    else
        disp('Unknown function handle!')
        return;
    end
end

[V,Vx] = computeEnergy(X,[],Vxf);

norm_Vx = sum(Vx.^2,1);
norm_x = sum(X.^2,1);

Vdot = sum(Vx.*Xd,1);
rho = rho0*(1-exp(-kappa0*norm_x)).*sqrt(norm_Vx);
ind = (Vdot + rho >= 0);
u = Xd*0;
if sum(ind)>0
    lambda = (Vdot(ind) + rho(ind))./norm_Vx(ind);
    u(:,ind) = -repmat(lambda,d,1).*Vx(:,ind);
%     [Vx Xd u]
    Xd(:,ind) = Xd(:,ind) + u(:,ind);
end

if ~isempty(varargin)
    dt = varargin{1};
    xn = X + Xd*dt;
    Vn = computeEnergy(xn,[],Vxf);
    ind = (Vn >= V);
    i = 0;
    while any(ind) && i<10
        alpha = V(ind)./Vn(ind);
%         alpha = (V(ind)-dt*rho(ind))./Vn(ind);
        Xd(:,ind) = repmat(alpha,d,1).*Xd(:,ind) - repmat(alpha.*sum(Xd(:,ind).*Vx(:,ind),1)./norm_Vx(ind),d,1).*Vx(:,ind);
        xn = X + Xd*dt;
        Vn = computeEnergy(xn,[],Vxf);
        ind = (Vn >= V);
%         ind = (Vn > V - dt*rho);
        i = i + 1;
    end
end
