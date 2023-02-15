function [status] = testmexampl(verbose,tol)
% TESTMEXAMPL tests that the behaviour of the AMPL-Matlab interface is correct
%
% RETURN
%   status ...  0 failed, 1 all ok
% INPUT
%   verbose ... 0 print only what failed, 1 print progress,
%               optional, default 1
%   tol ....... tolerance when comparing the results
%               optional, default 1e-8
%
% See also penlabtest, penlabstresstest
%
% TODO This needs improving!
%

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 5 Dec 2013

  if (nargin<2)
    tol = 1e-8;
  end
  if (nargin<1)
    verbose = 1;
  end
  status = 1;

  if (verbose)
    disp(' testing functionality of AMPL-mex interface...')
  end

  % is it possible to call it?
  try
     [x0,u0,ps] = amplf('examples/ex1.nl');
  catch
    disp(' Error: mexampl - cannot call')
    status = 0;
    return;
  end

  %status = status && onenleval(nlfile,x,v);
  status = status && onenleval('examples/ex1.nl',[0.1;0.2;-0.4],rand(2,1));
  status = status && onenleval('examples/ex3.nl',rand(2,1),rand(2,1));
  status = status && onenleval('datafiles/chain100.nl',rand(200,1),rand(101,1));
  status = status && onenleval('datafiles/israel.nl',rand(142,1),rand(163,1));

end

function [status] = onenleval(nlfile,x,v)
% ONENLEVAL one evaluation of AMPL (NL) problem at the given point
%   nlfile - input translated AMPL file
%   x - point where to evaluate
%   v - lagrangian multipliers for Hessian of the Lagrangian

  status = 1;

  % get the name of the test
  [path,name,ext] = fileparts(nlfile);
  %disp(name)

  % a crude test - just call everything and let's see
  try
    [x0, u0, ps] = amplf(nlfile,0);
    %length(x0)
    %length(u0)
    [fx, gx, hx] = amplf(x,0);
    [fdx, gdx, hdx] = amplf(x,1);
    ddL = amplf(v);
  catch
    disp([' test ' name ' FAILED'])
    status = 0;
  end

end

