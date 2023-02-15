function [prob]=alselftest(testID)
% ALSELFTEST is a self test of evaluation of the Augmented Lagrangian and its
% derivatives. It uses precomputed data from PenSDP and Pennon/Pennlp stored
% in data container file (DCF) in directory ./datafiles/
% Argument 'testID' is optional and if it is not present, it runs all the
% tests otherwise only these listed in vector 'testID'. Returns the last 
% loaded problem.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013
  
  prob = [];

  % hardcoded data - type, input1, input2, comment where input 1 and 2 will
  % be typically problem definition and matching DCF.
  tests = [
    {'pennlp','../../problems/ampl-nl/cosine.nl', 'datafiles/pennlp_cosine.dcf', 'big & unconstrained'}
    {'pennlp', '../../problems/ampl-nl/chain100.nl', 'datafiles/pennlp_chain100_a.dcf', '100 LIN eq + 1 NLN eq, no box (==> no penalties)'}
    {'pennlp', '../../problems/ampl-nl/israel.nl', 'datafiles/pennlp_israel_a.dcf', 'box + LIN ineq only (no equal)'}
    {'pennlp', '../../problems/ampl-nl/seba.nl', 'datafiles/pennlp_seba.dcf', 'box + LIN ineq & eq'}
    {'pennlp', '../../problems/ampl-nl/camshape100.nl', 'datafiles/pennlp_camshape100_a.dcf', 'NLN ineq + LIN eq + BOX'}

    {'pensdp', 'datafiles/truss1.dat-s', 'datafiles/pensdpa_truss1.dcf', 'very small matrices, no inequalities'} 
    {'pensdp', 'datafiles/theta1.dat-s', 'datafiles/pensdpa_theta1.dcf', 'one bigger matrix, no inequalitites'}
    {'pensdp', 'datafiles/control1.dat-s', 'datafiles/pensdpa_control1.dcf', 'two bigger matrices'}
    {'pensdp', 'datafiles/arch0.dat-s', 'datafiles/pensdpa_arch0.dcf', 'one matrix & set of inequalitites'}
    {'pensdp', 'datafiles/truss1b.dat-s', 'datafiles/pensdpa_truss1b.dcf', 'very small matrices, one inequality'} 
    ];

  testno=size(tests,1);

  if (nargin==0)
    testID=[1:testno];
  else
    testID=intersect(testID,[1:testno]);
  end

  for i=testID
    prob = runtest(i,tests{i,:});
  end
end
    
% run one test and return the object
function [prob] = runtest(ID, type, input1, input2, comment)

  [fdir,fname,fext] = fileparts(input1);
  disp(sprintf('* Test %2i - %s: %s, %s',ID,type,fname,comment));

  if (strcmpi(type,'pennlp') || strcmpi(type,'nlp'))

    penm=nlp_define(input1);
    prob=penlab(penm);
    [alx,aldx,alddx] = setuptestpoint(prob,input2,type);

  elseif (strcmpi(type,'pensdp') || strcmpi(type,'sdp'))

    sdpdata=readsdpa(input1);
    penm=sdp_define(sdpdata);
    prob=penlab(penm);
    [alx,aldx,alddx] = setuptestpoint(prob,input2,type);

  else
    disp('Unknown type of the test.');
    prob = [];
  end

  if (~isempty(prob))

    prob.eval_alx(); 
    prob.eval_aldx(); 
    prob.eval_alddx();

    prndiff('ALx', prob.ALx, alx);
    prndiff('ALdx', prob.ALdx, aldx);
    if (prob.Neq>0)
      prndiff('ALddx', prob.ALddx, alddx(1:prob.Nx,1:prob.Nx));
      prndiff('eqdx', prob.eqdx, alddx(1:prob.Nx,prob.Nx+1:end));
    else
      prndiff('ALddx', prob.ALddx, alddx);
    end

  end

end

% print differences (errors) absolute and relative
function prndiff(text,my,ref)
  diff=norm(my-ref,inf);
  nrm=norm(ref,inf);
  if (nrm==0)
    nrm=1;
  end

  if (diff/nrm>1e-10)
    wrn='   WARNING !!!';
  else
    wrn='';
  end
  disp(sprintf('  %-20s:   %e   %e  (abs|rel) %s',text,diff,diff/nrm,wrn));
  %if (diff/nrm>1e-10)
  %  disp('  !!!         WARNING         WARNING         WARNING         !!!');
  %end

end

