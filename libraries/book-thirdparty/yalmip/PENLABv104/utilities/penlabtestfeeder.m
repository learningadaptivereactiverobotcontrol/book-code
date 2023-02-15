function [status,name,prob,res] = penlabtestfeeder(no,testset)
% PENLABTESTFEEDER sets up one test to be run at a time.
%
% INPUT:
%   no - number of the test problem, 1..length(testset)
%   testset - (optional) is a list of all tests, their type
%        and the location of the datafile; if not set
%        the default selection of tests is used.
%
% OUTPUT:
%   status - 0  = test is set up;
%            <0 = problem with initialization of the requested test;
%            1  = wrong test number (maximal no of tests reached?)
%   name - name of the problem, this is used to show with the result
%   prob - if status==0, penlab structure with the test, otherwise []
%   res - expected result (objective function at the solution) or []
%
% Description of the testset: TODO (see to the default one below at the moment)
%
% See also penlabstresstest
%

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 3/12/2013

  if (nargin<1)
    no = 0;
  end

  if (nargin<2)
    % create the default testset
    % TODO later expand with (optional) expected result (e.g., the objective)
    testset = { ...
      { 'sdpa', 'datafiles/control1.dat-s', 1.778463e+01 }, ...
      { 'sdpa', 'datafiles/control2.dat-s', 8.300000e+00 }, ...
      { 'sdpa', 'datafiles/theta1.dat-s', 2.300000e+01 }, ...
      { 'sdpa', 'datafiles/theta2.dat-s', 3.287917e+01 }, ...
      { 'sdpa', 'datafiles/truss1.dat-s', -8.999996e+00 }, ...
      { 'sdpa', 'datafiles/arch0.dat-s', 5.66517e-01 }, ...
      { 'sdpa', 'datafiles/mcp100.dat-s', 2.261574e+02 }, ...
      { 'ampl', 'datafiles/camshape100.nl' }, ...
      { 'ampl', 'datafiles/chain100.nl' }, ...
      { 'ampl', 'datafiles/cosine.nl' }, ...
      { 'ampl', 'datafiles/israel.nl' }, ...
      { 'ampl', 'datafiles/polygon25.nl' }, ...
      { 'ampl', 'datafiles/seba.nl' }, ...
      { 'bmi', 'datafiles/bmi_example.mat' }, ...
      { 'bmi', 'datafiles/bmi_f4e.mat' }, ...
      { 'bmi', 'datafiles/bmi_interval1_0.mat' }, ...
      { 'bmi', 'datafiles/bmi_interval1_1.mat' }, ...
      { 'bmi', 'datafiles/bmi_interval2_0.mat' }, ...
      { 'bmi', 'datafiles/bmi_interval2_1.mat' }, ...
      { 'bmi', 'datafiles/bmi_jia.mat' }, ...
      { 'bmi', 'datafiles/bmi_patel5.mat' }, ...
      { 'bmi', 'datafiles/bmi_toy2.mat' }, ...
      { 'pmi', 'datafiles/pmi_example.mat' }, ...
      { 'pmi', 'datafiles/pmi_AC1.mat' }, ...
      { 'pmi', 'datafiles/pmi_NN4.mat' }, ...
    };
    % { 'bmi', 'datafiles/bmi_helicopter.mat' }, ...
    % { 'sdpa', 'datafiles/maxG11.dat-s' }, ...
    % { 'bmi', 'datafiles/bmi_toy1.mat' }, ...
  end

  ntests = length(testset);
  if (no<1 || no>ntests)
    status = 1;
    name = 'NONEXISTENT';
    prob = [];
    res = [];
    return;
  end

  test = testset{no};
  if (length(test)>=3)
    res = test{3};
  else
    res = [];
  end
  if (strcmpi(test{1},'sdpa'))
    [status,name,prob] = sdpa_feeder(test{2});
  elseif (strcmpi(test{1},'bmi'))
    [status,name,prob] = bmi_feeder(test{2});
  elseif (strcmpi(test{1},'pmi'))
    [status,name,prob] = pmi_feeder(test{2});
  elseif (strcmpi(test{1},'ampl'))
    [status,name,prob] = ampl_feeder(test{2});
  else
    % unknown test type
    status = -10;
    name = 'unknowntype';
    prob = [];
    res = [];
    return;
  end

end

function [status,name,prob] = sdpa_feeder(sdpafile)
% SDPA_FEEDER reads Linear SDP problem from a SDPA file into penlab object.
% Returns 
%   status~=0 if the file doesn't exist or there is a formating error,
%     or any other error with initialization, otherwise 0 if all OK
%   name name of the problem (basename of the SDPA data file)
%   prob penlab object
%

  if (isempty(sdpafile) || ~ischar(sdpafile))
    status = -2;
    name = 'EMPTY';
    prob = [];
    return;
  end

  [path,name,ext] = fileparts(sdpafile);

  try 
    sdpdata = readsdpa(sdpafile);
  catch
    status = -1;
    prob = [];
    return;
  end

  try
    penm = sdp_define(sdpdata);
    prob = penlab(penm);
    status = 0;
  catch
    status = -3;
    prob = [];
  end

end

function [status,name,prob] = bmi_feeder(bmifile)
% BMI_FEEDER loads bmidata from Matlab mat-file into penlab object.
% Returns 
%   status~=0 if the file doesn't exist or there is a formating error,
%     or any other error with initialization, otherwise 0 if all OK
%   name name of the problem (basename of the data file)
%   prob penlab object
%

  if (isempty(bmifile) || ~ischar(bmifile))
    status = -2;
    name = 'EMPTY';
    prob = [];
    return;
  end

  [path,name,ext] = fileparts(bmifile);

  try 
    % this mat-file should store bmidata struct
    load(bmifile);
  catch
    status = -1;
    prob = [];
    return;
  end

  try
    penm = bmi_define(bmidata);
    prob = penlab(penm);
    status = 0;
  catch
    status = -3;
    prob = [];
  end

end

function [status,name,prob] = pmi_feeder(pmifile)
% PMI_FEEDER loads pmidata from Matlab mat-file into penlab object.
% Returns 
%   status~=0 if the file doesn't exist or there is a formating error,
%     or any other error with initialization, otherwise 0 if all OK
%   name name of the problem (basename of the data file)
%   prob penlab object
%

  if (isempty(pmifile) || ~ischar(pmifile))
    status = -2;
    name = 'EMPTY';
    prob = [];
    return;
  end

  [path,name,ext] = fileparts(pmifile);

  try 
    % this mat-file should store pmidata struct
    load(pmifile);
  catch
    status = -1;
    prob = [];
    return;
  end

  try
    penm = pmi_define(pmidata);
    prob = penlab(penm);
    status = 0;
  catch
    status = -3;
    prob = [];
  end

end

function [status,name,prob] = ampl_feeder(amplfile)
% AMPL_FEEDER reads AMPL nl file and sets up a penlab object.
% Returns 
%   status~=0 if the file doesn't exist or there is a formating error,
%     or any other error with initialization, otherwise 0 if all OK
%   name name of the problem (basename of the data file)
%   prob penlab object
%

  if (isempty(amplfile) || ~ischar(amplfile))
    status = -2;
    name = 'EMPTY';
    prob = [];
    return;
  end

  [path,name,ext] = fileparts(amplfile);

  try 
    penm = nlp_define(amplfile);
  catch
    status = -1;
    prob = [];
    return;
  end

  try
    prob = penlab(penm);
    status = 0;
  catch
    status = -3;
    prob = [];
  end

end

