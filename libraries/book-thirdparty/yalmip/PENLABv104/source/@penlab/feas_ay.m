% Compute feasibility of the matrix constraints (A) and matrix variables (Y)
% Note that only these in pen/bar (non-strict treatment) are considered
% because these in the strict barrier are feasible by design.
% (This routine is called only during computation so any point must be 
% feasibile with respect to constraints in log-barrier. Similarly, it cannot 
% be more infeasible than the matrix penalty parameter.)
% TODO
% Suggested but not implemented...
%   nFast = 0 ... no speed up; if infeasible, go back with perturbation
%       to compute the feasibility up to PRECISION
%   nFast = 1 ... if infeasible, stop on first infeasible matrix constr.
%       and don't go back (==> the (in)feasibility is overestimated
%       up to SCALEUP factor times for this constraint and the other
%       are not checked --> could be even worse)
%   nFast = 2 ... if infeasible, don't go back with the perturbation
%       but at least check all the constraints that there is nothing
%       worse
%
%   how to make pstart&ptol dynamic to save some time...
%     + allow how to refine it-if it looks big, tolerance 1e-10 is time wasting
%   add changing order? start with the worst one from the last step??
%
function [mfeas] = feas_ay(obj)

  mfeas=0;
  pstart=1e-10;
  pstop=1e-7;

  % check only pen/bar; strict barrier must be feasible by design
  for k=obj.Yboxindphi

    pkx=obj.PYbox(k);            %2*p(sdpdata.Ng+k);
    Ykx = obj.Y{obj.Yboxmap(k)};
    M=obj.Yboxshift(k)*speye(size(Ykx)) + obj.Yboxmlt(k)*Ykx;
    % so far negative definite ... feasm is posdef checker
    %mfeas = feasm(-M, mfeas, Inf, pstart, pstop);
    mfeas = feasm(-M, mfeas, pkx, pstart, pstop);
  end

  for k=obj.Aindphi

    pkx=obj.PA(k);  % I used to use 2*         !!!!!!!!
    % TODO need to map the matrix first! - is it correct???
    kuser=obj.Amap(k);
    [Akuserx, obj.userdata] = obj.mconfun(obj.x, obj.Y, kuser, obj.userdata);
    M = obj.Ashift(k)*speye(size(Akuserx)) + obj.Amlt(k) .* Akuserx;
    %mfeas = feasm(-M, mfeas, Inf, pstart, pstop);
    mfeas = feasm(-M, mfeas, pkx, pstart, pstop);
  end


end


