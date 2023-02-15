% overloading default disp()
function disp(obj)

  fprintf('%s problem object\n',penlab.solvername)
  fprintf('  Problem name: %s\n',obj.probname);
  if (~isempty(obj.comment))
    fprintf('  Description:  %s\n',obj.comment);
  end
  switch(obj.phase)
    case 0  % empty
      str = 'problem hasn''t been loaded yet';
    case 1  % init
      str = 'problem initialized and loaded';
    case 2  % solving
      str = 'problem is being solved right now';
    case 3  % finished
      str = 'solver finished';
    otherwise
      str = sprintf('unnamed phase: %i',obj.phase);
  end
  fprintf('  Phase:        %s\n\n', str);

  if (obj.phase>0)
    nNLNineq=sum(obj.ineqmap<=obj.NgNLN);
    nNLNeq=sum(obj.eqmap<=obj.NgNLN);
    nNLNAineq=sum(obj.Amap<=obj.NANLN);
    fprintf('                              normal    mvars (m.elems)\n');
    fprintf('  Number of variables        %7d  %7d (%7d)\n',obj.Nx, obj.NY, obj.NYnnz);
    fprintf('                                 box   linear   nonlin\n');
    fprintf('  (Function) inequalities    %7d  %7d  %7d\n',obj.Nxbox,obj.Nineq-nNLNineq,nNLNineq);
    fprintf('  (Function) equalities               %7d  %7d\n',obj.Neq-nNLNeq, nNLNeq);
    fprintf('  Matrix     inequalities    %7d  %7d  %7d\n',obj.NYbox,obj.NA-nNLNAineq,nNLNAineq);
  end


  if (obj.phase>=2)
    if (obj.phase<3)
      fprintf('\n  Optimality meassures in the last completed iteration:\n');
    else
      fprintf('\n  Optimality meassures in the final iteration:\n');
    end
    fprintf('  Objective                %27.16E\n',obj.objx);
    fprintf('  Relative precision       %27.16E\n',abs(obj.ALx-obj.objx)/max(1,obj.objx));
    fprintf('  Compl. Slackness         %27.16E\n',obj.rCompl);
    fprintf('  Grad augm. lagr.         %27.16E\n',obj.rNormG);
    fprintf('  Feasibility              %27.16E\n',obj.rFeas);
    fprintf('  Minimal penalty          %27.16E\n',min([obj.pxbox;obj.pineq]));

    fprintf('  Newton steps                                   %5d\n',obj.miter);
    fprintf('  Inner steps                                    %5d\n',obj.initer);
    fprintf('  Linesearch steps                               %5d\n',obj.lsiter);
    %fprintf('  Time statistics:\n');
    %fprintf('     - total process time             %14g s\n',obj.stats_time_total);
    %fprintf('     - all minimization steps         %14g s\n',obj.stats_time_miters);
    %fprintf('     - all factorizations             %14g s\n',obj.stats_time_fact);
    %fprintf('     - function values evaluation     %14g s\n',obj.stats_time_alx);
    %fprintf('     - gradient values evaluation     %14g s\n',obj.stats_time_aldx);
    %fprintf('     - hessian values evaluation      %14g s\n',obj.stats_time_alddx);
  end

end
