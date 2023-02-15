function [rRelStep, nFlag]=ls_fullstep(obj, dir)
% rewritten line_search() @ line_search_els.c @ Pennlp v.2.3 & Pennon v.0.9
%
% changes in 'obj': xall, ALx, ALdx

  alp = 1;
  
  xall0=obj.xall;
  fx=obj.ALx;
  obj.xall = xall0 + alp*dir;
  obj.eval_alx();
    f = obj.ALx;
    nEff = 0;
  
      fx=f;
      obj.eval_aldx();
      
    obj.lsiter_last=obj.lsiter_last + 1;
  obj.print(3,Inf,'Step length: %f', alp);
  
  %return alp;
  rRelStep=alp;
  nFlag = 0;
  return;
  

