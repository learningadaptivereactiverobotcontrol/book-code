function [status] = penlabtest(verbose)
%PENLABTEST performs a couple of test to check if PenLab is working properly.
%
%  Please run PENLABTEST after unpacking the PenLab distribution package
%  and adding it to the matlab path (e.g., by calling install.m).
%
%  Input
%     verbose ... 0=be silent during the tests, otherwise be verbose;
%                 optional, default is silent
%  
%  Return value
%     0 ... one of major components/tests failed
%    -1 ... some parts outside the core are missing/failed
%     1 ... all test passed
%

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 5 Dec 2013

  if (nargin<1)
    verbose = 0;
  end 
  status = 1;
  have_ampl = 1;

  %%% check if all the main directories are on the path %%%
  disp('Checking if all components are present...')
  % main sources
  if (exist('boundschecker','file')~=2)
    disp('Error: penlab/source is not on the path.')
    status = 0;
  end
  if (exist('penlab','class')~=8)
    disp('Error: penlab/source is not on the path or penlab class is missing.')
    status = 0;
  end
  if (exist('readsdpa','file')~=2)
    disp('Error: penlab/utilities is not on the path.')
    status = 0;
  end
  % interfaces
  if (exist('bmi_define','file')~=2)
    disp('Error: penlab/interfaces/BMI is not on the path.')
    status = 0;
  end
  if (exist('nlp_define','file')~=2)
    disp('Error: penlab/interfaces/NLP_AMPL is not on the path.')
    status = 0;
  end
  if (exist('pmi_define','file')~=2)
    disp('Error: penlab/interfaces/PMI is not on the path.')
    status = 0;
  end
  if (exist('sdp_define','file')~=2)
    disp('Error: penlab/interfaces/LMI is not on the path.')
    status = 0;
  end
  % examples
  if (exist('ex1_define','file')~=2)
    disp('Error: penlab/examples is not on the path.')
    status = 0;
  end
  % doc_generator
  if (exist('m2html','file')~=2)
    disp('Warning: penlab/doc_generator is not on the path.')
    disp('... cannot generate internal documentation.')
    if (status~=0)
      status = -1;
    end
  end
  % directories
  if (exist('datafiles','dir')~=7)
    disp('Error: penlab/datafiles is missing.')
    status = 0;
  end
  if (exist('applications','dir')~=7)
    disp('Error: penlab/applications is missing.')
    status = 0;
  end
  disp(' ')

  if (status==0)
    disp('Some of the essential components are missing, cannot continue.');
    return
  end

  statuslabel = { 'FAILED', 'is ok' };

  %%% check if mex files are compiled for this system %%%
  disp('Checking if all mex files are compiled & work...')
  if (exist('mexsumsparse','file')~=3)
    disp('Error: mexsumsparse is not compiled.')
    disp('... please consult manual how to compile mex or how to run mex-free')
    status = 0;
  else
    ok = testmexsumsparse(verbose);
    disp(['mexsumsparse ' statuslabel{ok+1}])
    status = status * ok;
  end

  if (exist('mextrcolumn','file')~=3)
    disp('Error: mextrcolumn is not compiled.')
    disp('... please consult manual how to compile mex or how to run mex-free')
    status = 0;
  else
    ok = testmextrcolumn(verbose);
    disp(['mextrcolumn ' statuslabel{ok+1}])
    status = status * ok;
  end

  if (exist('mextrdsdsmat','file')~=3)
    disp('Error: mextrdsdsmat is not compiled.')
    disp('... please consult manual how to compile mex or how to run mex-free')
    status = 0;
  else
    ok = testmextrdsdsmat(verbose);
    disp(['mextrdsdsmat ' statuslabel{ok+1}])
    status = status * ok;
  end

  if (exist('amplf','file')~=3)
    disp('Warning: amplf is not compiled.')
    disp('... AMPL-NLP interface will not work')
    disp('... please consult manual how to compile mex')
    disp('... some of the tests will be skipped.')
    have_ampl = 0;
    if (status~=0)
      status = -1;
    end
  else
    ok = testmexampl(verbose);
    disp(['mexampl ' statuslabel{ok+1}])
    status = status * ok;
  end
  disp(' ')

  %%% check SDPA file reader %%%
  % this should be ideally improved
  disp('Checking if readsdpa() work...')
  try
    sdpdata = readsdpa('datafiles/control1.dat-s');
    sdpdata = readsdpa('datafiles/theta1.dat-s');
    sdpdata = readsdpa('datafiles/mcp100.dat-s');
  catch
    disp('Error: readsdpa does not seem to work properly')
    status = 0;
  end
  disp(' ')

  if (status==0)
    disp('Some of the essential components are missing, cannot continue.');
    return
  end

  %%% run a few problems with the inputs from various files %%%
  % this should be ideally improved
  disp('Running sample LMI, BMI and PMI problems...')
  testset = { ...
    { 'sdpa', 'datafiles/control1.dat-s', 1.778463e+01 }, ...
    { 'sdpa', 'datafiles/theta1.dat-s', 2.300000e+01 }, ...
    { 'sdpa', 'datafiles/truss1.dat-s', -8.999996e+00 }, ...
    { 'sdpa', 'datafiles/mcp100.dat-s', 2.261574e+02 }, ...
    { 'bmi', 'datafiles/bmi_example.mat' }, ...
    { 'bmi', 'datafiles/bmi_f4e.mat' }, ...
    { 'pmi', 'datafiles/pmi_example.mat' }, ...
    { 'pmi', 'datafiles/pmi_AC1.mat' }, ...
    { 'pmi', 'datafiles/pmi_NN4.mat' }, ...
  };
  feeder = @(no) penlabtestfeeder(no,testset);
  ok = runset(feeder,verbose);
  status = status * ok;
  disp(' ')

  if (have_ampl)
    disp('Running sample NLP-AMPL problems...')
    testset = { ...
      { 'ampl', 'datafiles/chain100.nl' }, ...
      { 'ampl', 'datafiles/israel.nl' }, ...
      { 'ampl', 'examples/ex3.nl' }, ...
    };
    feeder = @(no) penlabtestfeeder(no,testset);
    ok = runset(feeder,verbose);
    status = status * ok;
  else
    disp('SKIPPING sample NLP problems from AMPL')
  end
  disp(' ')

  % todo add more tests

end

function [ok] = runset(feeder,verbose,tol)
% RUNSET runs a series of tests on the solver.
%
% This works pretty much the same as penlabstresstest. The only
% difference is the output to the screen.
%
% The problems to test are generated by FEEDER. This routine executes them 
% and prints out one-liner statistics for comparison.
%
% INPUT:
%   feeder - routine returning the penlab objects to be ran,
%            optional, if not present, the default feeder is used;
%            it should look like:
%               function [status,name,prob,res] = feeder(no)
%            and to test number no=1..maxtests return status=0,
%            problem name and problem as penlab object,
%            expected results may be present in res, otherwise [],
%            if there are problems with the test, status<0,
%            status=1 means out of range of test
%
% See also penlabstresstest, penlabtestfeeder
%

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 5/12/2013

  % relative precision for comparison the objective?

  if (nargin<3)
    tol = 1e-4;
  end
  if (nargin<2)
    verbose = 1;
  end
  ok = 1;

  % run each test feeder returns
  no = 1;
  while (1)
    
    try
      [status,name,prob,res] = feeder(no);
    catch
      % problem with generating the test
      fprintf('%-20s: cannot generate (%i)\n',name,status);
      ok = 0;
    end

    if (status==1)
      % no more tests
      break;
    elseif (status==0)
      % all OK, run test

      try
        % set common option settings:
        if (verbose)
          prob.opts.outlev = 2;
        else
          prob.opts.outlev = 0;
        end
        prob.opts.outlev_file = 0;

        ifail = prob.solve();

        if (~isempty(res))
          if (abs(prob.objx - res)>tol*(abs(prob.objx)+abs(res)))
            fprintf('%-20s: not matching reference result %10.2e  %10.2e\n',...
              name,prob.objx,res);
            ok = 0;
          else
            fprintf('%-20s: ok\n', name);
          end
        else
          fprintf('%-20s: ok\n', name);
        end
      catch
        fprintf('%-20s: ERROR\n',name);
        ok = 0;
      end

      clear prob;

    end

    no = no + 1;

  end

end

